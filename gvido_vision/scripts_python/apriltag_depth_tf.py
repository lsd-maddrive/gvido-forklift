#!/usr/bin/env python3
"""
AprilTag детектор для OAK-D
- Показывает 2 окна: RGB и Depth
- Детектит теги
- Публикует TF в ROS
"""

import cv2
import numpy as np
import depthai as dai
from pyapriltags import Detector
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import threading
import time

# ============================================================
# НАСТРОЙКИ
# ============================================================
TAG_SIZE = 0.162  # 16.2 см
TAG_FAMILY = 'tag36h11'

# Параметры камеры (из калибровки)
CAMERA_PARAMS = {
    'fx': 634.37,
    'fy': 633.00,
    'cx': 280.28,
    'cy': 230.60,
    'baseline': 0.075
}

# ============================================================
# ROS TF ПАБЛИШЕР
# ============================================================
class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tag_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.get_logger().info("✅ TF Broadcaster запущен")
    
    def publish_tag(self, tag_id, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link_R"
        t.child_frame_id = f"tag_{tag_id}"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

# ============================================================
# ОСНОВНОЙ КЛАСС
# ============================================================
class DepthAprilTagDetector:
    def __init__(self):
        # Детектор AprilTag
        self.detector = Detector(families=TAG_FAMILY, nthreads=4)
        
        # Инициализация ROS
        rclpy.init()
        self.tf_pub = TFBroadcaster()
        self.ros_thread = threading.Thread(target=self._spin_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Настройка пайплайна
        self.setup_pipeline()
    
    def _spin_ros(self):
        while True:
            rclpy.spin_once(self.tf_pub, timeout_sec=0.1)
    
    def setup_pipeline(self):
        """Настройка OAK-D пайплайна"""
        self.pipeline = dai.Pipeline()
        
        # RGB камера
        self.cam_rgb = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        # Левый и правый сенсоры для глубины
        self.left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        
        # StereoDepth
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.stereo.setRectification(True)
        self.stereo.setExtendedDisparity(True)
        self.stereo.setLeftRightCheck(True)
        
        # Запрашиваем выходы
        self.rgbOut = self.cam_rgb.requestOutput(size=(640, 480), fps=30)
        self.leftOut = self.left.requestOutput(size=(640, 400), fps=30)
        self.rightOut = self.right.requestOutput(size=(640, 400), fps=30)
        
        # Связываем
        self.leftOut.link(self.stereo.left)
        self.rightOut.link(self.stereo.right)
        
        # Очереди
        self.rgbQueue = self.rgbOut.createOutputQueue()
        self.disparityQueue = self.stereo.disparity.createOutputQueue()
    
    def depth_to_meters(self, disparity, focal_length, baseline_mm):
        if disparity <= 0:
            return 0
        return (focal_length * baseline_mm) / disparity / 1000.0
    
    def calculate_3d_position(self, center, depth_m):
        if depth_m <= 0:
            return None
        u, v = center
        X = (u - CAMERA_PARAMS['cx']) * depth_m / CAMERA_PARAMS['fx']
        Y = (v - CAMERA_PARAMS['cy']) * depth_m / CAMERA_PARAMS['fy']
        Z = depth_m
        return (X, Y, Z)
    
    def run(self):
        print("🔌 Подключение к OAK-D камере...")
        print("   Окна: RGB и Depth")
        print("   TF публикуются в ROS (camera_link_R → tag_X)")
        print("   Нажмите 'q' для выхода\n")
        
        # Создаем окна
        cv2.namedWindow("RGB Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RGB Camera", 640, 480)
        cv2.namedWindow("Depth Map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Depth Map", 640, 480)
        
        with self.pipeline:
            self.pipeline.start()
            print("✅ Камера готова!\n")
            
            while self.pipeline.isRunning():
                in_rgb = self.rgbQueue.tryGet()
                in_disparity = self.disparityQueue.tryGet()
                
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    tags = self.detector.detect(gray)
                    
                    for tag in tags:
                        corners = tag.corners.astype(int)
                        center = tag.center.astype(int)
                        
                        # Рисуем контур тега
                        for i in range(4):
                            cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
                        cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"ID: {tag.tag_id}", 
                                   (center[0] - 20, center[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        
                        # Получаем глубину
                        depth_m = 0
                        if in_disparity is not None:
                            disp_frame = in_disparity.getFrame()
                            h, w = disp_frame.shape
                            xd = int(center[0] * w / frame.shape[1])
                            yd = int(center[1] * h / frame.shape[0])
                            if 0 <= xd < w and 0 <= yd < h:
                                disparity = disp_frame[yd, xd]
                                if disparity > 0:
                                    depth_m = self.depth_to_meters(disparity, 
                                        CAMERA_PARAMS['fx'], 
                                        CAMERA_PARAMS['baseline'] * 1000)
                        
                        # 3D позиция и публикация TF
                        if depth_m > 0:
                            pos = self.calculate_3d_position(center, depth_m)
                            if pos:
                                X, Y, Z = pos
                                cv2.putText(frame, f"Z: {Z:.2f}m", 
                                           (center[0] - 20, center[1] + 20),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                                
                                # Публикуем TF в ROS
                                self.tf_pub.publish_tag(tag.tag_id, X, Y, Z)
                                print(f"🔍 Тег {tag.tag_id}: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} м")
                        else:
                            cv2.putText(frame, "Z: N/A", 
                                       (center[0] - 20, center[1] + 20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    cv2.putText(frame, f"Tags: {len(tags)}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.imshow("RGB Camera", frame)
                
                # Показываем карту глубины
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
            print("\n👋 Завершено")

# ============================================================
# ТОЧКА ВХОДА
# ============================================================
if __name__ == "__main__":
    detector = DepthAprilTagDetector()
    detector.run()