import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct
import threading
import select
import sys
import termios
import tty

class PointCloudCalibrator(Node):
    def __init__(self):
        super().__init__('pointcloud_calibrator')
        
        self.left_to_right_transform = {
            'x': -0.02712694, 'y': -0.30988929, 'z': -0.00385942,
            'roll': 0.00959107, 'pitch': -0.00786656, 'yaw': 0.04171215
        }
        
        self.current_transform = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
        }
        
        self.calculate_initial_position()
        
        self.large_step = 0.1
        self.small_step = 0.01
        self.current_step_mode = 'small'
        
        self.base_frame = 'base_link'
        self.right_lidar_frame = 'rslidar_right'
        self.left_lidar_frame = 'rslidar_left'
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/right/rslidar_points',
            self.pointcloud_callback,
            10)
        
        self.publisher = self.create_publisher(
            PointCloud2,
            '/right/rslidar_points_calibrated',
            10)
        
        self.tf_timer = self.create_timer(0.1, self.publish_tf_transforms)
        
        self.running = True
        
        self.input_thread = threading.Thread(target=self.keyboard_listener)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info('PointCloud Calibrator initialized')
        self.print_help()

    def calculate_initial_position(self):
        """Автоматически вычисляет начальное положение base_link между лидарами"""
        v = np.array([
            self.left_to_right_transform['x'],
            self.left_to_right_transform['y'],
            self.left_to_right_transform['z']
        ], dtype=float)

        L = np.linalg.norm(v)
        if L < 1e-6:
            self.get_logger().warn("Лидары слишком близко: длина вектора ~0")
            v = np.array([0.0, 1.0, 0.0])
            L = 1.0

        v_hat = v / L
        target = np.array([0.0, 1.0, 0.0])

        axis = np.cross(v_hat, target)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-9:
            if np.dot(v_hat, target) > 0.0:
                rot = R.identity()
            else:
                aux = np.array([1.0, 0.0, 0.0])
                if abs(np.dot(v_hat, aux)) > 0.99:
                    aux = np.array([0.0, 0.0, 1.0])
                axis = np.cross(v_hat, aux)
                axis = axis / np.linalg.norm(axis)
                rot = R.from_rotvec(axis * np.pi)
        else:
            axis = axis / axis_norm
            angle = np.arccos(np.clip(np.dot(v_hat, target), -1.0, 1.0))
            rot = R.from_rotvec(axis * angle)

        yaw, pitch, roll = rot.as_euler('zyx', degrees=False)

        self.current_transform = {
            'x': 0.0,
            'y': -L/2.0,
            'z': 0.0,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }

        self.get_logger().info(
            f"Автоустановка: L={L:.3f} м; base→right=(0,-{L/2:.3f},0), "
            f"ось Y(base) направлена в точности в центр rslidar_left."
        )

    def print_help(self):
        """Выводит справку по управлению"""
        help_text = f"""
=== Управление калибровкой ===
Текущий шаг: {self.current_step_mode} ({self.get_current_step():.3f})

Управление позицией правого лидара (относительно base_link):
  u/o - X/Z +шаг
  j/l - X/Z -шаг

Управление углами (относительно base_link):
  q/w/e - roll/pitch/yaw +шаг
  a/s/d - roll/pitch/yaw -шаг

Переключение шага:
  m - переключение между большим ({self.large_step:.3f}) и малым ({self.small_step:.3f}) шагом

Сброс к автоматическому положению:
  r - сброс к автоматически вычисленному положению

Показать текущие значения:
  p - показать параметры

Выход:
  Ctrl+C - выход из программы

Текущие значения (правый лидар относительно base_link):
"""
        self.get_logger().info(help_text)
        self.print_current_values()

    def get_current_step(self):
        """Возвращает текущий шаг регулировки"""
        return self.small_step if self.current_step_mode == 'small' else self.large_step

    def print_current_values(self):
        """Выводит текущие значения параметров"""
        current = self.current_transform
        
        info = f"""  Позиция: x={current['x']:.3f}, z={current['z']:.3f}
  Углы: roll={np.degrees(current['roll']):.1f}°, pitch={np.degrees(current['pitch']):.1f}°, yaw={np.degrees(current['yaw']):.1f}°
  Шаг: {self.current_step_mode} ({self.get_current_step():.3f})
  
  Автоматически вычислено:
  - base_link посередине между лидарами
  - Ось Y base_link направлена от правого лидара к левому
"""
        self.get_logger().info(info)

    def keyboard_listener(self):
        """Поток для чтения клавиш из терминала"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.handle_keypress(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_keypress(self, key):
        """Обрабатывает нажатия клавиш"""
        step = self.get_current_step()
        
        if key == 'u':
            self.adjust_position('x', step)
        elif key == 'j':
            self.adjust_position('x', -step)
        elif key == 'o':
            self.adjust_position('z', step)
        elif key == 'l':
            self.adjust_position('z', -step)
        elif key == 'q':
            self.adjust_angle('roll', step)
        elif key == 'a':
            self.adjust_angle('roll', -step)
        elif key == 'w':
            self.adjust_angle('pitch', step)
        elif key == 's':
            self.adjust_angle('pitch', -step)
        elif key == 'e':
            self.adjust_angle('yaw', step)
        elif key == 'd':
            self.adjust_angle('yaw', -step)
        elif key == 'm':
            self.toggle_step_mode()
        elif key == 'r':
            self.reset_to_auto_position()
        elif key == 'p':
            self.print_current_values()
        elif key == '\x03':
            self.get_logger().info("Выход из программы...")
            self.running = False
            rclpy.shutdown()

    def adjust_position(self, axis: str, delta: float):
        """Изменяет указанную позицию на заданную величину"""
        self.current_transform[axis] += delta
        axis_name = {'x': 'X', 'z': 'Z'}[axis]
        direction = "увеличена" if delta > 0 else "уменьшена"
        self.get_logger().info(f"Позиция {axis_name} {direction} на {abs(delta):.3f} м")
        self.print_current_values()

    def adjust_angle(self, angle_type: str, delta: float):
        """Изменяет указанный угол на заданную величину"""
        self.current_transform[angle_type] += delta
        angle_name = {'roll': 'Roll', 'pitch': 'Pitch', 'yaw': 'Yaw'}[angle_type]
        direction = "увеличен" if delta > 0 else "уменьшен"
        self.get_logger().info(f"{angle_name} {direction} на {abs(np.degrees(delta)):.1f}°")
        self.print_current_values()

    def toggle_step_mode(self):
        """Переключает режим шага между большим и малым"""
        if self.current_step_mode == 'small':
            self.current_step_mode = 'large'
        else:
            self.current_step_mode = 'small'
        
        self.get_logger().info(f"Режим шага изменен на: {self.current_step_mode} ({self.get_current_step():.3f})")
        self.print_current_values()

    def reset_to_auto_position(self):
        """Сбрасывает к автоматически вычисленному положению"""
        self.calculate_initial_position()
        self.get_logger().info("Параметры сброшены к автоматически вычисленному положению")
        self.print_current_values()

    def publish_tf_transforms(self):
        """Публикует TF преобразования для лидаров"""
        rotation = R.from_euler('zyx', [
            self.current_transform['yaw'],
            self.current_transform['pitch'],
            self.current_transform['roll']
        ])

        translation_vector = np.array([
            self.current_transform['x'],
            self.current_transform['y'],
            self.current_transform['z']
        ])

        t_right = TransformStamped()
        t_right.header.stamp = self.get_clock().now().to_msg()
        t_right.header.frame_id = self.base_frame
        t_right.child_frame_id = self.right_lidar_frame
        t_right.transform.translation.x = float(translation_vector[0])
        t_right.transform.translation.y = float(translation_vector[1])
        t_right.transform.translation.z = float(translation_vector[2])
        q = rotation.as_quat()
        t_right.transform.rotation.x = q[0]
        t_right.transform.rotation.y = q[1]
        t_right.transform.rotation.z = q[2]
        t_right.transform.rotation.w = q[3]

        t_left = TransformStamped()
        t_left.header.stamp = t_right.header.stamp
        t_left.header.frame_id = self.right_lidar_frame
        t_left.child_frame_id = self.left_lidar_frame
        t_left.transform.translation.x = self.left_to_right_transform['x']
        t_left.transform.translation.y = self.left_to_right_transform['y']
        t_left.transform.translation.z = self.left_to_right_transform['z']
        rot_left = R.from_euler('zyx', [
            self.left_to_right_transform['yaw'],
            self.left_to_right_transform['pitch'],
            self.left_to_right_transform['roll']
        ])
        ql = rot_left.as_quat()
        t_left.transform.rotation.x = ql[0]
        t_left.transform.rotation.y = ql[1]
        t_left.transform.rotation.z = ql[2]
        t_left.transform.rotation.w = ql[3]

        self.tf_broadcaster.sendTransform([t_right, t_left])

    def create_transform_matrix(self, transform: dict) -> np.ndarray:
        """Создает матрицу преобразования из параметров трансформации"""
        rot = R.from_euler('zyx', [transform['yaw'], transform['pitch'], transform['roll']])
        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = np.array([transform['x'], transform['y'], transform['z']])
        return T

    def transform_pointcloud(self, pointcloud: PointCloud2, transform_matrix: np.ndarray) -> PointCloud2:
        """Применяет преобразование к облаку точек"""
        points = self.pointcloud2_to_array(pointcloud)
        
        if points.size == 0:
            return pointcloud
            
        homogeneous_points = np.ones((points.shape[0], 4))
        homogeneous_points[:, :3] = points[:, :3]
        transformed_points = (transform_matrix @ homogeneous_points.T).T
        
        transformed_pc = PointCloud2()
        transformed_pc.header = pointcloud.header
        transformed_pc.header.frame_id = self.base_frame
        transformed_pc.height = pointcloud.height
        transformed_pc.width = pointcloud.width
        transformed_pc.fields = pointcloud.fields
        transformed_pc.is_bigendian = pointcloud.is_bigendian
        transformed_pc.point_step = pointcloud.point_step
        transformed_pc.row_step = pointcloud.row_step
        transformed_pc.is_dense = pointcloud.is_dense
        
        transformed_pc.data = self.array_to_pointcloud2_data(transformed_points[:, :3], points[:, 3], pointcloud)
        
        return transformed_pc

    def pointcloud2_to_array(self, cloud: PointCloud2) -> np.ndarray:
        """Конвертирует PointCloud2 в numpy массив [x, y, z, intensity]"""
        x_offset = -1
        y_offset = -1
        z_offset = -1
        intensity_offset = -1
        
        for field in cloud.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
            elif field.name == 'intensity':
                intensity_offset = field.offset
        
        if x_offset == -1 or y_offset == -1 or z_offset == -1:
            self.get_logger().error('Required fields (x, y, z) not found in pointcloud')
            return np.array([])
        
        points = []
        for i in range(cloud.width * cloud.height):
            point_data = cloud.data[i * cloud.point_step:(i + 1) * cloud.point_step]
            
            try:
                x = struct.unpack('f', point_data[x_offset:x_offset + 4])[0]
                y = struct.unpack('f', point_data[y_offset:y_offset + 4])[0]
                z = struct.unpack('f', point_data[z_offset:z_offset + 4])[0]
                
                intensity = 0.0
                if intensity_offset != -1:
                    intensity = struct.unpack('f', point_data[intensity_offset:intensity_offset + 4])[0]
                
                points.append([x, y, z, intensity])
            except:
                continue
        
        return np.array(points)

    def array_to_pointcloud2_data(self, points: np.ndarray, intensities: np.ndarray, original_cloud: PointCloud2) -> bytes:
        """Конвертирует numpy массив обратно в данные PointCloud2"""
        data = bytearray()
        
        for i in range(points.shape[0]):
            point_bytes = bytearray(original_cloud.point_step)
            
            for j, coord in enumerate(['x', 'y', 'z']):
                offset = -1
                for field in original_cloud.fields:
                    if field.name == coord:
                        offset = field.offset
                        break
                
                if offset != -1:
                    struct.pack_into('f', point_bytes, offset, points[i, j])
            
            intensity_offset = -1
            for field in original_cloud.fields:
                if field.name == 'intensity':
                    intensity_offset = field.offset
                    break
            
            if intensity_offset != -1 and i < len(intensities):
                struct.pack_into('f', point_bytes, intensity_offset, intensities[i])
            
            data.extend(point_bytes)
        
        return bytes(data)

    def pointcloud_callback(self, msg: PointCloud2):
        """Обработчик входящих облаков точек"""
        transform_matrix = self.create_transform_matrix(self.current_transform)
        
        calibrated_pc = self.transform_pointcloud(msg, transform_matrix)
        
        self.publisher.publish(calibrated_pc)

    def destroy_node(self):
        """Переопределяем для корректного завершения"""
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    calibrator = PointCloudCalibrator()
    
    try:
        while rclpy.ok() and calibrator.running:
            rclpy.spin_once(calibrator, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()