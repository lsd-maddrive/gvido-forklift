#!/usr/bin/env python3
"""
OAK-D: Depth + AprilTag детекция + Публикация TF в ROS
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
from tf2_ros import TransformBroadcaster

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
        
        # --- Детектор AprilTag (pyapriltags) ---
        # quad_decimate: чем больше, тем быстрее, но ниже точность
        # nthreads: количество потоков для обработки
        self.detector = Detector(
            families=TAG_FAMILY, 
            nthreads=8,            # 8 потоков для скорости
            quad_decimate=4.0,     # Уменьшаем изображение в 4 раза = быстрее
            refine_edges=0,        # Отключаем уточнение краёв = быстрее
            decode_sharpening=0.0  # Отключаем
        )
        
        # --- Настройка пайплайна OAK-D ---
        self.setup_pipeline()
        
        # --- ROS спиннер в отдельном потоке (чтобы не блокировать основной цикл) ---
        self.ros_thread = threading.Thread(target=self._spin_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()

        # --- Сглаживание позиции тега (фильтр скользящего среднего) ---
        self.smooth_buffer = []        # Буфер для хранения последних измерений
        self.smooth_window = 10        # Количество измерений для усреднения (больше = плавнее, но больше задержка)
        
        # --- Минимальное изменение для публикации TF (чтобы не спамить при малых движениях) ---
        self.last_published = None     # Последняя опубликованная позиция
        self.min_change = 0.01         # 1 см — публикуем только если изменение больше
    
    # ------------------------------------------------------------------------
    # ROS спиннер (обрабатывает колбэки ROS)
    # ------------------------------------------------------------------------
    def _spin_ros(self):
        while True:
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
    
    # ------------------------------------------------------------------------
    # Публикация трансформации в ROS (/tf)
    # ------------------------------------------------------------------------
    def publish_tf(self, tag_id, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.ros_node.get_clock().now().to_msg()
        t.header.frame_id = "camera_link_R"      # Родительский frame
        t.child_frame_id = f"tag_{tag_id}"       # Дочерний frame (тег)
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.w = 1.0             # Без поворота (кватернион)
        self.tf_broadcaster.sendTransform(t)
    
    # ------------------------------------------------------------------------
    # Настройка пайплайна OAK-D (RGB + стерео + глубина)
    # ------------------------------------------------------------------------
    def setup_pipeline(self):
        self.pipeline = dai.Pipeline()
        
        # RGB камера (CAM_A)
        self.cam_rgb = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        # Левая и правая камеры для стерео (CAM_B и CAM_C)
        self.left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        
        # Узел стерео глубины
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.stereo.setRectification(True)       # Выпрямление изображений
        self.stereo.setExtendedDisparity(True)   # Увеличенная дальность
        self.stereo.setLeftRightCheck(True)      # Проверка левый-правый для фильтрации шума
        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # Выравнивание глубины под RGB
        self.stereo.setOutputSize(640, 480)      # Размер выходного изображения глубины
        
        # Запрашиваем потоки с камер
        self.rgbOut = self.cam_rgb.requestOutput(size=(640, 480), fps=30)
        self.leftOut = self.left.requestOutput(size=(640, 400), fps=30)
        self.rightOut = self.right.requestOutput(size=(640, 400), fps=30)
        
        # Связываем левую и правую камеры со стерео узлом
        self.leftOut.link(self.stereo.left)
        self.rightOut.link(self.stereo.right)
        
        # Очереди для получения данных
        self.rgbQueue = self.rgbOut.createOutputQueue()       # RGB изображения
        self.depthQueue = self.stereo.depth.createOutputQueue()   # Глубина в мм
        self.disparityQueue = self.stereo.disparity.createOutputQueue()  # Диспаратность (для цветной карты)
    
    # ------------------------------------------------------------------------
    # Расчет 3D позиции тега относительно камеры
    # ------------------------------------------------------------------------
    def calculate_3d_position(self, center, depth_m):
        """
        center: (u, v) координаты центра тега в пикселях
        depth_m: глубина в метрах
        Возвращает (X, Y, Z) в метрах
        """
        if depth_m <= 0:
            return None
        u, v = center
        # X и Y поменяны местами для правильной ориентации
        X = (v - CAMERA_PARAMS['cy']) * depth_m / CAMERA_PARAMS['fy']
        Y = (u - CAMERA_PARAMS['cx']) * depth_m / CAMERA_PARAMS['fx']
        Z = depth_m
        return (X, Y, Z)
    
    # ------------------------------------------------------------------------
    # Основной цикл: захват кадров, детекция тегов, публикация TF
    # ------------------------------------------------------------------------
    def run(self):
        print("🔌 Connecting to OAK-D...")
        print("🎮 Controls: 'q' = quit")
        print("📡 TF публикуются в ROS: camera_link_R -> tag_X")
        
        with self.pipeline:
            self.pipeline.start()
            print("✅ Camera ready!")
            
            while self.pipeline.isRunning():
                # Получаем кадры из очередей
                in_rgb = self.rgbQueue.tryGet()
                in_depth = self.depthQueue.tryGet()
                in_disparity = self.disparityQueue.tryGet()
                
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    
                    # Замер времени детекции
                    start_time = time.time()
                    tags = self.detector.detect(gray)
                    detect_ms = (time.time() - start_time) * 1000
                    print(f"⚡ Детекция: {detect_ms:.0f} мс, тегов: {len(tags)}")
                    
                    # Обрабатываем каждый найденный тег
                    for tag in tags:
                        corners = tag.corners.astype(int)
                        center = tag.center.astype(int)
                        
                        # Рисуем контур тега (зеленый) и центр (красный)
                        for i in range(4):
                            cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
                        cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"ID: {tag.tag_id}", 
                                   (center[0] - 20, center[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        
                        # Получаем глубину в центре тега
                        depth_m = 0
                        if in_depth is not None:
                            depth_frame = in_depth.getFrame()
                            h, w = depth_frame.shape
                            xd = int(center[0] * w / frame.shape[1])
                            yd = int(center[1] * h / frame.shape[0])
                            if 0 <= xd < w and 0 <= yd < h:
                                depth_mm = depth_frame[yd, xd]
                                if depth_mm > 0:
                                    depth_m = depth_mm / 1000.0
                        
                        if depth_m > 0:
                            # Вычисляем 3D позицию
                            pos = self.calculate_3d_position(center, depth_m)
                            if pos:
                                X, Y, Z = pos
                                cv2.putText(frame, f"Z: {Z:.2f}m", (center[0] - 20, center[1] + 20),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                                
                                # ============================================
                                # СГЛАЖИВАНИЕ (фильтр скользящего среднего)
                                # ============================================
                                # Добавляем новое измерение в буфер
                                self.smooth_buffer.append((X, Y, Z))
                                
                                # Ограничиваем размер буфера
                                if len(self.smooth_buffer) > self.smooth_window:
                                    self.smooth_buffer.pop(0)
                                
                                # Вычисляем среднее арифметическое
                                avg_X = sum(p[0] for p in self.smooth_buffer) / len(self.smooth_buffer)
                                avg_Y = sum(p[1] for p in self.smooth_buffer) / len(self.smooth_buffer)
                                avg_Z = sum(p[2] for p in self.smooth_buffer) / len(self.smooth_buffer)
                                
                                # ============================================
                                # ПУБЛИКУЕМ TF ТОЛЬКО ПРИ ЗНАЧИТЕЛЬНОМ ИЗМЕНЕНИИ
                                # ============================================
                                if self.last_published is None:
                                    # Первая публикация
                                    self.last_published = (avg_X, avg_Y, avg_Z)
                                    self.publish_tf(tag.tag_id, avg_X, avg_Y, avg_Z)
                                else:
                                    # Проверяем изменение
                                    dx = abs(avg_X - self.last_published[0])
                                    dy = abs(avg_Y - self.last_published[1])
                                    dz = abs(avg_Z - self.last_published[2])
                                    
                                    # Публикуем только если изменение больше 1 см
                                    if dx > self.min_change or dy > self.min_change or dz > self.min_change:
                                        self.last_published = (avg_X, avg_Y, avg_Z)
                                        self.publish_tf(tag.tag_id, avg_X, avg_Y, avg_Z)
                                
                                # Выводим сглаженную позицию в консоль
                                print(f"🔍 Tag {tag.tag_id}: X={avg_X:.3f}, Y={avg_Y:.3f}, Z={avg_Z:.3f} m")
                        else:
                            cv2.putText(frame, "Z: N/A", (center[0] - 20, center[1] + 20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # Отображаем количество тегов на кадре
                    cv2.putText(frame, f"Tags: {len(tags)}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.imshow("AprilTag Detection", frame)
                
                # Отображаем цветную карту глубины (диспаратность)
                if in_disparity is not None:
                    disp_frame = in_disparity.getFrame()
                    max_disp = np.max(disp_frame)
                    if max_disp > 0:
                        depth_vis = (disp_frame / max_disp * 255).astype(np.uint8)
                    else:
                        depth_vis = disp_frame.astype(np.uint8)
                    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    cv2.imshow("Depth Map", depth_color)
                
                # Выход по клавише 'q'
                if cv2.waitKey(1) == ord('q'):
                    self.pipeline.stop()
                    break
        
        cv2.destroyAllWindows()


# ============================================================
# ТОЧКА ВХОДА
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    detector = DepthAprilTagDetector()
    detector.run()

if __name__ == "__main__":
    main()