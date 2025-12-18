import os
import sys
import subprocess
import json
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QLabel, QMessageBox, QFrame, 
                             QGridLayout, QLineEdit, QGroupBox)
from PyQt5.QtCore import pyqtSlot, QMetaObject, Qt, Q_ARG
from PyQt5.QtGui import QFont, QColor

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct

import tf2_ros


class PointCloudCalibrator(Node):
    def __init__(self):
        super().__init__('pointcloud_calibrator')
        
        self.left_to_right_transform = {
            'x': -0.03306870, 'y': -0.29416455, 'z': 0.00470458,
            'roll': 0.00959107, 'pitch': -0.00786656, 'yaw': 0.04171215
        }
        self.current_transform = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
        }
        
        self.calculate_initial_position()
        
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

        self.latest_transform = None
        self.transform_callback = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.tf_check_timer = self.create_timer(0.1, self.check_transform)

        self.get_logger().info('PointCloud Calibrator initialized')

    def set_transform_callback(self, callback):
        self.transform_callback = callback
    
    def check_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'rslidar_right', rclpy.time.Time()
            )
            self.latest_transform = transform
            if self.transform_callback:
                self.transform_callback(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
            pass
        
    def calculate_initial_position(self):
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

    def adjust_position(self, axis: str, delta: float):
        self.current_transform[axis] += delta

    def adjust_angle(self, angle_type: str, delta: float):
        self.current_transform[angle_type] += delta

    def publish_tf_transforms(self):
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
        rot = R.from_euler('zyx', [transform['yaw'], transform['pitch'], transform['roll']])
        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = np.array([transform['x'], transform['y'], transform['z']])
        return T

    def transform_pointcloud(self, pointcloud: PointCloud2, transform_matrix: np.ndarray) -> PointCloud2:
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
        transform_matrix = self.create_transform_matrix(self.current_transform)
        
        calibrated_pc = self.transform_pointcloud(msg, transform_matrix)
        
        self.publisher.publish(calibrated_pc)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

class TransformControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.rviz_process = None
        try:
            package_share_dir = get_package_share_directory('forklift_vision')
            self.config_path = os.path.join(package_share_dir, 'rviz', 'dual_rslidar.rviz')
        except Exception as e:
            self.config_path = os.path.join(os.path.expanduser("~"), 
                                          'ros2_ws', 'src', 'ros2-forklift-amr', 
                                          'high_level', 'forklift_vision', 
                                          'rviz', 'dual_rslidar.rviz')
        self.values = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0
        }
        self.steps = {
            'position': 0.1,
            'rotation': 0.1
        }
        self.init_ui()

        self.calibrator_node = None
        self.init_ros()
        
    def init_ui(self):
        self.setWindowTitle("Forklift Transform Controller")
        self.setGeometry(100, 100, 600, 700)
        
        central_widget = QWidget()
        central_widget.setStyleSheet("""
            QWidget {
                background-color: #2c3e50;
            }
        """)
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        title_label = QLabel("🎮 Управление трансформациями")
        title_label.setFont(QFont("Arial", 18, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("""
            QLabel {
                color: white;
                padding: 15px;
                background-color: #3498db;
                border-radius: 10px;
                margin-bottom: 10px;
            }
        """)
        main_layout.addWidget(title_label)
        
        grid_layout = QGridLayout()
        grid_layout.setSpacing(15)
        grid_layout.setContentsMargins(10, 10, 10, 10)
        
        headers = ["Параметр", "Текущее значение", "Управление"]
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setFont(QFont("Arial", 12, QFont.Bold))
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("""
                QLabel {
                    background-color: #34495e;
                    color: white;
                    padding: 8px;
                    border-radius: 5px;
                }
            """)
            grid_layout.addWidget(label, 0, col)
        
        parameters = [
            ('X (м)', 'x', 'position'),
            ('Y (м)', 'y', 'position'),
            ('Z (м)', 'z', 'position'),
            ('Yaw (рад)', 'yaw', 'rotation'),
            ('Pitch (рад)', 'pitch', 'rotation'),
            ('Roll (рад)', 'roll', 'rotation')
        ]
        
        self.value_displays = {}
        
        for row, (label_text, param, step_type) in enumerate(parameters, 1):
            param_label = QLabel(label_text)
            param_label.setFont(QFont("Arial", 11, QFont.Bold))
            param_label.setStyleSheet("color: white;")
            grid_layout.addWidget(param_label, row, 0)
            
            value_display = QLineEdit("0.000")
            value_display.setReadOnly(True)
            value_display.setAlignment(Qt.AlignCenter)
            value_display.setFont(QFont("Arial", 11))
            value_display.setStyleSheet("""
                QLineEdit {
                    background-color: #ffffff;
                    border: 2px solid #bdc3c7;
                    border-radius: 8px;
                    padding: 8px;
                    selection-background-color: #3498db;
                }
            """)
            grid_layout.addWidget(value_display, row, 1)
            self.value_displays[param] = value_display
            
            button_layout = QHBoxLayout()
            button_layout.setSpacing(10)
            
            minus_btn = QPushButton("-")
            minus_btn.setFixedSize(50, 40)
            minus_btn.clicked.connect(lambda checked, p=param, s=step_type: self.decrement_value(p, s))
            minus_btn.setStyleSheet(self.get_control_button_style("#e74c3c"))
            button_layout.addWidget(minus_btn)
            
            plus_btn = QPushButton("+")
            plus_btn.setFixedSize(50, 40)
            plus_btn.clicked.connect(lambda checked, p=param, s=step_type: self.increment_value(p, s))
            plus_btn.setStyleSheet(self.get_control_button_style("#27ae60"))
            button_layout.addWidget(plus_btn)
            
            button_widget = QWidget()
            button_widget.setLayout(button_layout)
            grid_layout.addWidget(button_widget, row, 2)
        
        main_layout.addLayout(grid_layout)
        
        step_group = QGroupBox("📏 Настройка шагов")
        step_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 12px;
                border: 2px solid #7f8c8d;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
                color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: white;
            }
        """)
        step_layout = QHBoxLayout(step_group)
        
        step_layout.addWidget(QLabel("Шаг позиции (м):"))
        self.pos_step_edit = QLineEdit(str(self.steps['position']))
        self.pos_step_edit.setMaximumWidth(80)
        self.pos_step_edit.textChanged.connect(self.update_position_step)
        self.pos_step_edit.setStyleSheet("""
            QLineEdit {
                background-color: white;
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                padding: 4px;
            }
        """)
        step_layout.addWidget(self.pos_step_edit)
        
        step_layout.addWidget(QLabel("Шаг угла (рад):"))
        self.rot_step_edit = QLineEdit(str(self.steps['rotation']))
        self.rot_step_edit.setMaximumWidth(80)
        self.rot_step_edit.textChanged.connect(self.update_rotation_step)
        self.rot_step_edit.setStyleSheet("""
            QLineEdit {
                background-color: white;
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                padding: 4px;
            }
        """)
        step_layout.addWidget(self.rot_step_edit)
        
        step_layout.addStretch()
        main_layout.addWidget(step_group)

        tf_group = QGroupBox("🔄 Трансформация base_link → rslidar_right")
        tf_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 12px;
                border: 2px solid #7f8c8d;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
                color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: white;
            }
        """)
        tf_layout = QVBoxLayout(tf_group)
        
        self.tf_display = QLineEdit()
        self.tf_display.setReadOnly(True)
        self.tf_display.setFont(QFont("Courier", 9))
        self.tf_display.setStyleSheet("""
            QLineEdit {
                background-color: #1a1a1a;
                color: #00ff00;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 8px;
                font-family: 'Courier New';
            }
        """)
        self.tf_display.setMinimumHeight(60)
        tf_layout.addWidget(QLabel("Формат static_transform_publisher:"))
        
        tf_display_layout = QHBoxLayout()
        tf_display_layout.addWidget(self.tf_display)

        self.copy_tf_btn = QPushButton("📋")
        self.copy_tf_btn.setFixedWidth(40)
        self.copy_tf_btn.setToolTip("Копировать код трансформации")
        self.copy_tf_btn.clicked.connect(lambda: self.copy_to_clipboard(self.tf_display.text()))
        self.copy_tf_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                border-radius: 4px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        tf_display_layout.addWidget(self.copy_tf_btn)

        tf_display_widget = QWidget()
        tf_display_widget.setLayout(tf_display_layout)
        tf_layout.addWidget(tf_display_widget)
        
        self.tf_command_display = QLineEdit()
        self.tf_command_display.setReadOnly(True)
        self.tf_command_display.setFont(QFont("Courier", 9))
        self.tf_command_display.setStyleSheet("""
            QLineEdit {
                background-color: #1a1a1a;
                color: #00ffff;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 8px;
                font-family: 'Courier New';
            }
        """)
        self.tf_command_display.setMinimumHeight(40)
        tf_layout.addWidget(QLabel("Команда ros2 run:"))
        
        tf_command_layout = QHBoxLayout()
        tf_command_layout.addWidget(self.tf_command_display)

        self.copy_command_btn = QPushButton("📋")
        self.copy_command_btn.setFixedWidth(40)
        self.copy_command_btn.setToolTip("Копировать команду")
        self.copy_command_btn.clicked.connect(lambda: self.copy_to_clipboard(self.tf_command_display.text()))
        self.copy_command_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                border-radius: 4px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        tf_command_layout.addWidget(self.copy_command_btn)

        tf_command_widget = QWidget()
        tf_command_widget.setLayout(tf_command_layout)
        tf_layout.addWidget(tf_command_widget)
        
        main_layout.addWidget(tf_group)

        rviz_group = QGroupBox("🖥️ Управление RViz2")
        rviz_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 12px;
                border: 2px solid #7f8c8d;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
                color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: white;
            }
        """)
        rviz_layout = QHBoxLayout(rviz_group)
        
        self.start_rviz_btn = QPushButton("🚀 Запустить RViz2")
        self.start_rviz_btn.clicked.connect(self.start_rviz2)
        self.start_rviz_btn.setStyleSheet(self.get_button_style("#27ae60"))
        rviz_layout.addWidget(self.start_rviz_btn)
        
        self.stop_rviz_btn = QPushButton("🛑 Остановить RViz2")
        self.stop_rviz_btn.clicked.connect(self.stop_rviz2)
        self.stop_rviz_btn.setEnabled(False)
        self.stop_rviz_btn.setStyleSheet(self.get_button_style("#e74c3c"))
        rviz_layout.addWidget(self.stop_rviz_btn)
        
        main_layout.addWidget(rviz_group)
        
        self.status_label = QLabel("Готов к работе")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #95a5a6;
                color: white;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        main_layout.addWidget(self.status_label)
        
        self.update_displays()
        
    def get_button_style(self, color, font_size=12):
        return f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                padding: 12px 20px;
                border-radius: 8px;
                font-weight: bold;
                font-size: {font_size}px;
                min-height: 25px;
            }}
            QPushButton:hover {{
                background-color: {self.darken_color(color)};
            }}
            QPushButton:pressed {{
                background-color: {self.darken_color(color, 40)};
            }}
            QPushButton:disabled {{
                background-color: #bdc3c7;
                color: #7f8c8d;
            }}
        """
    
    def get_control_button_style(self, color):
        return f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                border-radius: 6px;
                font-weight: bold;
                font-size: 16px;
            }}
            QPushButton:hover {{
                background-color: {self.darken_color(color)};
            }}
            QPushButton:pressed {{
                background-color: {self.darken_color(color, 40)};
            }}
        """
    
    def darken_color(self, color, percent=20):
        color = QColor(color)
        return color.darker(100 + percent).name()
    
    def increment_value(self, param, step_type):
        step = self.steps[step_type]
        self.values[param] += step
        self.update_display(param)
        self.update_status(f"Увеличено {param}: {self.values[param]:.3f}")
        
        self.update_calibrator(param, step)
    
    def decrement_value(self, param, step_type):
        step = self.steps[step_type]
        self.values[param] -= step
        self.update_display(param)
        self.update_status(f"Уменьшено {param}: {self.values[param]:.3f}")
        
        self.update_calibrator(param, -step)
    
    def update_calibrator(self, param, delta):
        if self.calibrator_node is None:
            return
            
        try:
            if param in ['x', 'y', 'z']:
                self.calibrator_node.adjust_position(param, delta)
            elif param in ['roll', 'pitch', 'yaw']:
                self.calibrator_node.adjust_angle(param, delta)
        except Exception as e:
            pass
    
    def update_display(self, param):
        value = self.values[param]
        self.value_displays[param].setText(f"{value:.3f}")
    
    def update_displays(self):
        for param, display in self.value_displays.items():
            self.update_display(param)
    
    def update_position_step(self):
        try:
            self.steps['position'] = float(self.pos_step_edit.text())
        except ValueError:
            pass
    
    def update_rotation_step(self):
        try:
            self.steps['rotation'] = float(self.rot_step_edit.text())
        except ValueError:
            pass
    
    def update_status(self, message):
        self.status_label.setText(message)

    def copy_to_clipboard(self, text):
        clipboard = QApplication.clipboard()
        clipboard.setText(text)
        self.update_status("Текст скопирован в буфер обмена")

    def init_ros(self):
        try:
            rclpy.init()
            self.calibrator_node = PointCloudCalibrator()
            
            self.values = self.calibrator_node.current_transform.copy()
            
            self.calibrator_node.set_transform_callback(self.on_transform_received)
            
            self.update_displays()
            
            import threading
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
        except Exception as e:
            pass

    def ros_spin(self):
        while rclpy.ok() and self.calibrator_node.running:
            rclpy.spin_once(self.calibrator_node, timeout_sec=0.1)

    def on_transform_received(self, transform):
        from PyQt5.QtCore import QMetaObject, Qt, Q_ARG
        QMetaObject.invokeMethod(self, "_update_tf_display", 
                               Qt.QueuedConnection,
                               Q_ARG(object, transform))
    
    @pyqtSlot(object)
    def _update_tf_display(self, transform):
        try:
            t = transform.transform.translation
            r = transform.transform.rotation
            
            quat_x = -r.x
            quat_y = -r.y  
            quat_z = -r.z
            quat_w = -r.w
            
            tf_command_text = (f"ros2 run tf2_ros static_transform_publisher \\\n"
                            f"    {t.x:.6f} {t.y:.6f} {t.z:.6f} \\\n"
                            f"    {quat_x:.6f} {quat_y:.6f} {quat_z:.6f} {quat_w:.6f} \\\n"
                            f"    base_link rslidar_right")
            
            tf_node_text = (f"static_tf_base_link_rslidar_right = Node(\n"
                        f"    package='tf2_ros',\n"
                        f"    executable='static_transform_publisher',\n"
                        f"    arguments=[\n"
                        f"        '{t.x:.6f}',"
                        f"        '{t.y:.6f}',"
                        f"        '{t.z:.6f}',"
                        f"        '{quat_x:.6f}',"
                        f"        '{quat_y:.6f}',"
                        f"        '{quat_z:.6f}',"
                        f"        '{quat_w:.6f}',\n"
                        f"        # Frames\n"
                        f"        'base_link',"
                        f"        'rslidar_right'"
                        f"    ],\n"
                        f"    parameters=[{{\"use_sim_time\": use_sim_time}}]\n"
                        f")")
            
            if self.tf_display:
                self.tf_display.setText(tf_node_text)
            
            if self.tf_command_display:
                self.tf_command_display.setText(tf_command_text)
                
        except Exception as e:
            pass

    def start_rviz2(self):
        if not os.path.exists(self.config_path):
            QMessageBox.critical(self, "Ошибка", f"Файл конфигурации не найден: {self.config_path}")
            return
        
        try:
            cmd = ['rviz2', '-d', self.config_path]
            self.rviz_process = subprocess.Popen(cmd)
            
            self.start_rviz_btn.setEnabled(False)
            self.stop_rviz_btn.setEnabled(True)
            self.update_status("RViz2 запущен в отдельном окне")
            
        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f"Не удалось запустить RViz2: {e}")
            self.update_status("Ошибка запуска RViz2")
    
    def stop_rviz2(self):
        if self.rviz_process:
            self.rviz_process.terminate()
            try:
                self.rviz_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.rviz_process.kill()
            
            self.start_rviz_btn.setEnabled(True)
            self.stop_rviz_btn.setEnabled(False)
            self.update_status("RViz2 остановлен")
    
    def closeEvent(self, event):
        self.stop_rviz2()
        try:
            rclpy.shutdown()
        except:
            pass
        event.accept()

def main():
    app = QApplication(sys.argv)
    
    app.setStyle('Fusion')
    
    try:
        subprocess.run(['which', 'rviz2'], capture_output=True, check=True)
    except subprocess.CalledProcessError:
        QMessageBox.critical(None, "Ошибка", "RViz2 не найден. Убедитесь, что ROS2 установлен и настроен.")
        return
    
    window = TransformControlApp()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()