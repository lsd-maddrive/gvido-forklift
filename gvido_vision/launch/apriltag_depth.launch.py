#!/usr/bin/env python3
"""
Launch файл для Python AprilTag детектора (с DepthAI API)
+ стабилизатор тегов
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Путь к пакету
    pkg_share = FindPackageShare('gvido_vision').find('gvido_vision')
    
    # ============================================================
    # 1. Статическая трансформация: base_link -> camera_link_R
    # ============================================================
    static_tf_base_to_camR = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camR',
        arguments=['0.0', '0.2328', '1.245', '-1.5917', '0.0201', '-1.5821', 'base_link', 'camera_link_R'],
        output='screen',
    )
    
    # ============================================================
    # 2. Статическая трансформация: camera_link_R -> camera_link_L
    # ============================================================
    static_tf_camR_to_camL = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camR_to_camL',
        arguments=['-0.4654', '-0.0095', '-0.0096', '0.0', '0.0', '0.0', 'camera_link_R', 'camera_link_L'],
        output='screen',
    )
    
    # ============================================================
    # 3. Python AprilTag детектор
    # ============================================================
    apriltag_node = Node(
        package='gvido_vision',
        executable='depth_apriltag_ros',
        name='depth_apriltag_ros',
        output='screen',
    )
    
    # ============================================================
    # 4. Фильтр стабилизации тегов (усредняет позиции)
    # ============================================================
    tag_filter = Node(
        package='gvido_vision',
        executable='stereo_filter',
        name='tag_tf_stabilizer',
        parameters=[os.path.join(pkg_share, 'config', 'tag_filter.yaml')],
        output='screen',
    )
    
    return LaunchDescription([
        static_tf_base_to_camR,
        # static_tf_camR_to_camL,
        apriltag_node,
        # tag_filter,
    ])