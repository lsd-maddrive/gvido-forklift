#!/usr/bin/env python3
"""
AprilTag детектор для ROS топика с OAK-D камеры
Подписывается на /oak/rgb/image_raw и /oak/stereo/depth
Публикует TF с реальной глубиной
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pyapriltags import Detector
import numpy as np
import cv_bridge

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        
        # Параметры
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oak/stereo/depth')
        self.declare_parameter('camera_frame', 'camera_link_R')
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_size', 0.162)
        
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_family = self.get_parameter('tag_family').value
        self.tag_size = self.get_parameter('tag_size').value
        
        # Параметры камеры (из lcalib.yaml)
        self.camera_params = (634.37, 633.00, 280.28, 230.60)
        
        # TF бродкастер
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Создаем детектор AprilTag
        self.detector = Detector(
            families=self.tag_family,
            nthreads=4,
            quad_decimate=1.0,
            refine_edges=1
        )
        
        # CV Bridge для конвертации ROS изображений
        self.bridge = cv_bridge.CvBridge()
        
        # Храним последнюю глубину
        self.last_depth = None
        
        # Подписка на топик изображения
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub = self.create_subscription(
            Image, 
            self.image_topic, 
            self.image_callback, 
            qos
        )
        
        # Подписка на топик глубины
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos
        )
        
        self.get_logger().info(f"✅ AprilTag детектор инициализирован")
        self.get_logger().info(f"   Подписан на: {self.image_topic}")
        self.get_logger().info(f"   Depth topic: {self.depth_topic}")
        self.get_logger().info(f"   Camera frame: {self.camera_frame}")
    
    def depth_callback(self, msg):
        """Сохраняем последнюю глубину"""
        try:
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            self.get_logger().info(f"✅ Глубина получена! shape={self.last_depth.shape}")
        except Exception as e:
            self.get_logger().error(f"Ошибка глубины: {e}")
    
    def publish_tag_tf(self, tag, position_3d):
        """Публикует трансформацию camera → tag в /tf"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = f"tag_{tag.tag_id}"
        
        t.transform.translation.x = position_3d[0]
        t.transform.translation.y = position_3d[1]
        t.transform.translation.z = position_3d[2]
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def calculate_3d_position(self, center, depth_m):
        """Вычисляет 3D позицию тега на основе глубины"""
        if depth_m <= 0:
            return None
        u, v = center
        fx, fy = self.camera_params[0], self.camera_params[1]
        cx, cy = self.camera_params[2], self.camera_params[3]
        X = (u - cx) * depth_m / fx
        Y = (v - cy) * depth_m / fy
        Z = depth_m
        return (X, Y, Z)
    
    def image_callback(self, msg):
        """Обработка изображения из топика"""
        if self.last_depth is None:
            self.get_logger().warn("Нет данных глубины, ждем...")
            return
        self.get_logger().info(f"📏 Глубина есть, shape={self.last_depth.shape}")
        try:
            # Конвертируем ROS изображение в OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Детекция тегов
            tags = self.detector.detect(gray)
            
            # Получаем размеры глубины
            depth_h, depth_w = self.last_depth.shape
            frame_h, frame_w = frame.shape[:2]
            
            for tag in tags:
                center = tag.center
                center_int = (int(center[0]), int(center[1]))
                
                # Находим соответствующий пиксель глубины
                x_depth = int(center[0] * depth_w / frame_w)
                y_depth = int(center[1] * depth_h / frame_h)
                
                if 0 <= x_depth < depth_w and 0 <= y_depth < depth_h:
                    depth_m = self.last_depth[y_depth, x_depth] / 1000.0  # мм -> м
                else:
                    depth_m = 0
                
                if depth_m > 0:
                    # Вычисляем 3D позицию
                    pos_3d = self.calculate_3d_position(center, depth_m)
                    if pos_3d:
                        X, Y, Z = pos_3d
                        self.publish_tag_tf(tag, pos_3d)
                        self.get_logger().info(f"🔍 Тег {tag.tag_id}: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} м")
                else:
                    # Fallback: оценка по размеру тега
                    corners = tag.corners
                    width_px = np.linalg.norm(corners[0] - corners[1])
                    if width_px > 0:
                        distance = (self.tag_size * frame_w) / width_px
                        pos_3d = [0, 0, distance]
                        self.publish_tag_tf(tag, pos_3d)
                        self.get_logger().info(f"🔍 Тег {tag.tag_id}: (приблизительно) Z={distance:.3f} м")
            
            # Визуализация (опционально, для отладки)
            for tag in tags:
                corners = tag.corners.astype(int)
                for i in range(4):
                    cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
                center = tuple(tag.center.astype(int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {tag.tag_id}", 
                           (center[0] - 20, center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            cv2.imshow("AprilTag Detection", frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()