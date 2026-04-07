#!/usr/bin/env python3
"""
AprilTag детектор для OAK-D камеры в реальном времени
Использует pyapriltags библиотеку
"""

import cv2
import numpy as np
import depthai as dai
from pyapriltags import Detector
import time

class OAKAprilTagDetector:
    def __init__(self, families='tag36h11', tag_size=0.162, quad_decimate=1.0):
        """
        Инициализация детектора
        
        Args:
            families: семейство тегов ('tag36h11', 'tag25h9', и т.д.)
            tag_size: реальный размер тега в метрах
            quad_decimate: уменьшение разрешения для ускорения
        """
        # Создаем детектор AprilTag
        self.detector = Detector(
            families=families,
            nthreads=4,
            quad_decimate=quad_decimate,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        self.tag_size = tag_size
        self.families = families
        
        # Параметры камеры (если есть калибровка - загрузить)
        self.camera_params = None  # [fx, fy, cx, cy]
        
        print(f"✅ Детектор AprilTag инициализирован")
        print(f"   Семейство: {families}")
        print(f"   Размер тега: {tag_size} м")
    
    def setup_oak_pipeline(self):
        """Настройка пайплайна OAK-D камеры"""
        pipeline = dai.Pipeline()
        
        # Узел RGB камеры
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)
        
        # Узел для вывода данных
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)
        
        print("✅ OAK-D пайплайн настроен")
        return pipeline
    
    def detect_tags(self, frame):
        """Детекция тегов на кадре"""
        # Конвертируем в gray (AprilTag работает с черно-белым)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Детекция тегов
        tags = self.detector.detect(
            gray,
            #estimate_tag_pose=False,  # Пока без оценки позы
            #camera_params=None,
            #tag_size=self.tag_size
        )
        
        return tags
    
    def draw_tags(self, frame, tags):
        """Рисуем найденные теги на кадре"""
        for tag in tags:
            # Рисуем контур тега
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
            
            # Рисуем центр тега
            center = tuple(tag.center.astype(int))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            
            # Пишем ID тега
            cv2.putText(frame, f"ID: {tag.tag_id}", 
                       (center[0] - 20, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Пишем позицию (если есть)
            if hasattr(tag, 'pose_t') and tag.pose_t is not None:
                pos = tag.pose_t
                cv2.putText(frame, f"X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}",
                           (center[0] - 20, center[1] + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        return frame
    
    def run(self):
        """Запуск детекции в реальном времени"""
        # Настройка пайплайна
        pipeline = self.setup_oak_pipeline()
        
        # Подключение к камере
        print("🔌 Подключение к OAK-D камере...")
        
        try:
            with dai.Device(pipeline) as device:
                print("✅ Камера подключена!")
                
                # Получаем очередь вывода
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                
                # Статистика FPS
                fps = 0
                frame_count = 0
                start_time = time.time()
                
                print("\n🎥 Начало детекции. Нажмите 'q' для выхода\n")
                
                while True:
                    # Получаем кадр
                    in_frame = q_rgb.tryGet()
                    
                    if in_frame is not None:
                        # Конвертируем в OpenCV формат
                        frame = in_frame.getCvFrame()
                        
                        # Детекция тегов
                        tags = self.detect_tags(frame)
                        
                        # Рисуем результаты
                        frame = self.draw_tags(frame, tags)
                        
                        # Показываем FPS
                        frame_count += 1
                        if time.time() - start_time >= 1.0:
                            fps = frame_count
                            frame_count = 0
                            start_time = time.time()
                        
                        cv2.putText(frame, f"FPS: {fps}", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        cv2.putText(frame, f"Tags found: {len(tags)}", (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        
                        # Показываем кадр
                        cv2.imshow("OAK-D AprilTag Detector", frame)
                        
                        # Вывод в консоль
                        if len(tags) > 0:
                            for tag in tags:
                                print(f"🔍 Найден тег ID: {tag.tag_id}, Центр: {tag.center}")
                    
                    # Выход по 'q'
                    if cv2.waitKey(1) == ord('q'):
                        break
        
        except Exception as e:
            print(f"❌ Ошибка: {e}")
        
        finally:
            cv2.destroyAllWindows()
            print("\n👋 Программа завершена")

def main():
    print("=" * 50)
    print("AprilTag Detector для OAK-D камеры")
    print("=" * 50)
    
    # Создаем детектор
    detector = OAKAprilTagDetector(
        families='tag36h11',  # Семейство тегов
        tag_size=0.162,       # Размер 16.2 см
        quad_decimate=1.0     # Без уменьшения разрешения
    )
    
    # Запускаем
    detector.run()

if __name__ == "__main__":
    main()