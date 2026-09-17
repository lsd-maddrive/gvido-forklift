#!/usr/bin/env python3
"""
OAK-D: Depth + AprilTag детекция + Публикация TF в ROS
+ Публикация RGB, Depth и CameraInfo топиков для RTAB-Map
"""

import cv2
import numpy as np
import depthai as dai
from pyapriltags import Detector
import math
import threading
import time
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from collections import defaultdict

# ============================================================
# НАСТРОЙКИ
# ============================================================
TAG_SIZE = 0.162          # Размер тега в метрах (16.2 см)
TAG_FAMILY = 'tag36h11'   # Семейство тегов

# Параметры калибровки камеры (fx, fy, cx, cy)
CAMERA_PARAMS = {
    'fx': 634.37, 'fy': 633.00,
    'cx': 280.28, 'cy': 230.60,
    'baseline': 0.075      # Расстояние между камерами (база) в метрах
}

# ============================================================
# ОСНОВНОЙ КЛАСС
# ============================================================
class DepthAprilTagDetector:
    def __init__(self):
        # --- Инициализация ROS ---
        self.ros_node = rclpy.create_node('apriltag_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self.ros_node)
        self.bridge = CvBridge()
        
        # --- Публикаторы для RTAB-Map ---
        self.rgb_pub = self.ros_node.create_publisher(Image, '/camera_left/image_raw', 10)
        self.depth_pub = self.ros_node.create_publisher(Image, '/camera_left/depth/image_raw', 10)
        self.camera_info_pub = self.ros_node.create_publisher(CameraInfo, '/camera_left/camera_info', 10)
        
        # --- Детектор AprilTag ---
        self.detector = Detector(
            families=TAG_FAMILY, 
            nthreads=8,
            quad_decimate=4.0,
            refine_edges=0,
            decode_sharpening=0.0
        )
        
        # --- Настройка пайплайна OAK-D ---
        self.setup_pipeline()
        
        # --- ROS спиннер ---
        self.ros_thread = threading.Thread(target=self._spin_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()

        # --- Отдельные буферы для каждого тега ---
        self.smooth_buffers = defaultdict(list)
        self.smooth_window = 20
        self.last_published = {}
        self.min_change = 0.05
        
        # --- CameraInfo сообщение (заполняется один раз) ---
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_link_R"
        self.camera_info_msg.height = 480
        self.camera_info_msg.width = 640
        self.camera_info_msg.distortion_model = "plumb_bob"
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.k = [CAMERA_PARAMS['fx'], 0.0, CAMERA_PARAMS['cx'],
                                   0.0, CAMERA_PARAMS['fy'], CAMERA_PARAMS['cy'],
                                   0.0, 0.0, 1.0]
        self.camera_info_msg.r = [1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0]
        self.camera_info_msg.p = [CAMERA_PARAMS['fx'], 0.0, CAMERA_PARAMS['cx'], 0.0,
                                   0.0, CAMERA_PARAMS['fy'], CAMERA_PARAMS['cy'], 0.0,
                                   0.0, 0.0, 1.0, 0.0]
    
    # ------------------------------------------------------------------------
    def _spin_ros(self):
        while True:
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
    
    # ------------------------------------------------------------------------
    def publish_tf(self, tag_id, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.ros_node.get_clock().now().to_msg()
        t.header.frame_id = "camera_link_R"
        t.child_frame_id = f"tag_{tag_id}"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.ros_node.get_logger().info(f"📡 Опубликован TF: tag_{tag_id} -> ({x:.3f}, {y:.3f}, {z:.3f})")
    
    # ------------------------------------------------------------------------
    def setup_pipeline(self):
        self.pipeline = dai.Pipeline()
        
        self.cam_rgb = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        self.left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.stereo.setRectification(True)
        self.stereo.setSubpixel(True)
        self.stereo.setExtendedDisparity(True)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        self.stereo.setOutputSize(640, 480)
        
        self.rgbOut = self.cam_rgb.requestOutput(size=(640, 480), fps=30)
        self.leftOut = self.left.requestOutput(size=(640, 400), fps=30)
        self.rightOut = self.right.requestOutput(size=(640, 400), fps=30)
        
        self.leftOut.link(self.stereo.left)
        self.rightOut.link(self.stereo.right)
        
        self.rgbQueue = self.rgbOut.createOutputQueue()
        self.depthQueue = self.stereo.depth.createOutputQueue()
        self.disparityQueue = self.stereo.disparity.createOutputQueue()
    
    # ------------------------------------------------------------------------
    def calculate_3d_position(self, center, depth_m):
        if depth_m <= 0:
            return None
        u, v = center
        X = (v - CAMERA_PARAMS['cy']) * depth_m / CAMERA_PARAMS['fy']
        Y = (u - CAMERA_PARAMS['cx']) * depth_m / CAMERA_PARAMS['fx']
        Z = depth_m
        return (X, Y, Z)
    
    # ------------------------------------------------------------------------
    def publish_rgb_depth_info(self, rgb_frame, depth_frame, stamp):
        """Публикует RGB, Depth и CameraInfo топики для RTAB-Map"""
        
        # Публикуем RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, "bgr8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = "camera_link_R"
        self.rgb_pub.publish(rgb_msg)
        
        # Публикуем Depth (в метрах, float32)
        if depth_frame is not None:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame.astype(np.float32), "32FC1")
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = "camera_link_R"
            self.depth_pub.publish(depth_msg)
        
        # Публикуем CameraInfo
        self.camera_info_msg.header.stamp = stamp
        self.camera_info_pub.publish(self.camera_info_msg)
    
    # ------------------------------------------------------------------------
    def run(self):
        print("🔌 Connecting to OAK-D...")
        print("🎮 Controls: 'q' = quit")
        print("📡 TF публикуются в ROS: camera_link_R -> tag_X")
        print("📡 RGB и Depth топики публикуются для RTAB-Map")
        
        with self.pipeline:
            self.pipeline.start()
            print("✅ Camera ready!")
            
            while self.pipeline.isRunning():
                in_rgb = self.rgbQueue.tryGet()
                in_depth = self.depthQueue.tryGet()
                in_disparity = self.disparityQueue.tryGet()
                
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    stamp = self.ros_node.get_clock().now().to_msg()
                    
                    # Получаем depth frame (если есть)
                    depth_frame = None
                    if in_depth is not None:
                        depth_frame = in_depth.getFrame().astype(np.float32) / 1000.0  # мм -> метры
                    
                    # Публикуем RGB, Depth, CameraInfo для RTAB-Map
                    self.publish_rgb_depth_info(frame, depth_frame, stamp)
                    
                    # Детекция AprilTag
                    start_time = time.time()
                    tags = self.detector.detect(gray)
                    detect_ms = (time.time() - start_time) * 1000
                    print(f"⚡ Детекция: {detect_ms:.0f} мс, тегов: {len(tags)}")
                    
                    for tag in tags:
                        corners = tag.corners.astype(int)
                        center = tag.center.astype(int)
                        
                        for i in range(4):
                            cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
                        cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"ID: {tag.tag_id}", 
                                   (center[0] - 20, center[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        
                        # Получаем глубину в центре тега
                        depth_m = 0
                        if depth_frame is not None:
                            h, w = depth_frame.shape
                            xd = int(center[0] * w / frame.shape[1])
                            yd = int(center[1] * h / frame.shape[0])
                            if 0 <= xd < w and 0 <= yd < h:
                                depth_m = depth_frame[yd, xd]
                        
                        if depth_m > 0:
                            pos = self.calculate_3d_position(center, depth_m)
                            if pos:
                                X, Y, Z = pos
                                cv2.putText(frame, f"Z: {Z:.2f}m", (center[0] - 20, center[1] + 20),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                                
                                tag_id = tag.tag_id
                                self.smooth_buffers[tag_id].append((X, Y, Z))
                                
                                if len(self.smooth_buffers[tag_id]) > self.smooth_window:
                                    self.smooth_buffers[tag_id].pop(0)
                                
                                buffer = self.smooth_buffers[tag_id]
                                avg_X = sum(p[0] for p in buffer) / len(buffer)
                                avg_Y = sum(p[1] for p in buffer) / len(buffer)
                                avg_Z = sum(p[2] for p in buffer) / len(buffer)
                                
                                self.publish_tf(tag_id, avg_X, avg_Y, avg_Z)
                                self.last_published[tag_id] = (avg_X, avg_Y, avg_Z)
                                print(f"🔍 Tag {tag_id}: X={avg_X:.3f}, Y={avg_Y:.3f}, Z={avg_Z:.3f} m")
                        else:
                            cv2.putText(frame, "Z: N/A", (center[0] - 20, center[1] + 20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    cv2.putText(frame, f"Tags: {len(tags)}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.imshow("AprilTag Detection", frame)
                
                if in_disparity is not None:
                    disp_frame = in_disparity.getFrame()
                    max_disp = np.max(disp_frame)
                    if max_disp > 0:
                        depth_vis = (disp_frame / max_disp * 255).astype(np.uint8)
                    else:
                        depth_vis = disp_frame.astype(np.uint8)
                    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    cv2.imshow("Depth Map", depth_color)
                
                if cv2.waitKey(1) == ord('q'):
                    self.pipeline.stop()
                    break
        
        cv2.destroyAllWindows()

# ============================================================
def main(args=None):
    rclpy.init(args=args)
    detector = DepthAprilTagDetector()
    detector.run()

if __name__ == "__main__":
    main()