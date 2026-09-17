#!/usr/bin/env python3
"""
Launch файл для полной системы:
- Статические трансформации
- AprilTag детектор (OAK-D)
- RTAB-Map (официальный launch) с поддержкой тегов
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # ============================================================
    # 1. Статическая трансформация: base_link -> camera_link_R
    # ============================================================
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_cam',
        arguments=['0.0', '0.2328', '1.245', '-1.5917', '0.0201', '-1.5821', 'base_link', 'camera_link_R'],
        output='screen',
    )
    
    # ============================================================
    # 2. AprilTag детектор (твой)
    # ============================================================
    detector = Node(
        package='gvido_vision',
        executable='depth_apriltag_ros',
        name='depth_apriltag_ros',
        output='screen',
    )
    
    # ============================================================
    # 3. Официальный RTAB-Map launch (с приоритетами тегов)
    # ============================================================
    
    # Путь к карте тегов
    tag_map_path = os.path.expanduser('~/ros2_ws/src/gvido-forklift/gvido_vision/maps/tag_map.yaml')
    
    # Читаем координаты тегов из карты
    import yaml
    priors = ""
    if os.path.exists(tag_map_path):
        with open(tag_map_path, 'r') as f:
            data = yaml.safe_load(f)
            if data and 'tags' in data:
                tag_list = []
                for tag_id, info in data['tags'].items():
                    tag_num = tag_id.replace('tag_', '')
                    x = info['translation']['x']
                    y = info['translation']['y']
                    tag_list.append(f"{tag_num} {x} {y} 0 0 0 0")
                priors = ' '.join(tag_list)
    
    # Формируем аргументы RTAB-Map
    rtabmap_args = f"--delete_db_on_start --Marker/Priors \"{priors}\" --Optimizer/PriorsIgnored false"
    
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            )
        ),
        launch_arguments={
            # Твои топики
            'rgb_topic': '/camera_left/image_raw',
            'depth_topic': '/camera_left/depth/image_raw',
            'camera_info_topic': '/camera_left/camera_info',
            'frame_id': 'base_link',
            # Включение визуальной одометрии
            'visual_odometry': 'true',
            # Синхронизация
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.05',
            'topic_queue_size': '50',
            'sync_queue_size': '50',
            # TF
            'publish_tf_map': 'true',
            # Визуализация
            'rtabmap_viz': 'false',
            'rviz': 'true',
            # Твои теги для коррекции
            'tag_topic': '/apriltag/detections',
            # Аргументы (приоритеты тегов)
            'rtabmap_args': rtabmap_args,
        }.items()
    )
    
    return LaunchDescription([
        static_tf,
        detector,
        rtabmap,
    ])