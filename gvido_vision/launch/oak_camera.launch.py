#!/usr/bin/env python3
"""
Launch файл для OAK-D камеры + AprilTag детектор + стабилизатор тегов
Всё в одном!
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Путь к пакету
    pkg_share = FindPackageShare('gvido_vision').find('gvido_vision')
    
    # Аргументы
    oak_ns_arg = DeclareLaunchArgument('oak_ns', default_value='oak')
    camera_frame_arg = DeclareLaunchArgument('camera_frame', default_value='camera_link_R')
    
    # Путь к конфигу камеры
    config_file = os.path.join(pkg_share, 'config', 'oak_camera.yaml')
    
    # ============================================================
    # 1. OAK-D камера (RGBD с выравниванием)
    # ============================================================
    oak_camera = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak',
        namespace=LaunchConfiguration('oak_ns'),
        parameters=[config_file],
        output='screen',
    )
    
    # ============================================================
    # 2. Статическая трансформация base_link -> camera_link_R
    # ============================================================
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'camera_link_R'],
        output='screen',
    )
    
    # ============================================================
    # 3. AprilTag детектор (подписывается на RGB и Depth)
    # ============================================================
    apriltag_detector = Node(
        package='gvido_vision',
        executable='apriltag_detector',
        name='apriltag_detector',
        output='screen',
    )
    
    # Задержка для детектора (ждем инициализации камеры)
    apriltag_delayed = TimerAction(
        period=3.0,
        actions=[apriltag_detector]
    )
    
    # ============================================================
    # 4. Фильтр стабилизации тегов (stereo_filter.py)
    # ============================================================
    tag_filter = Node(
        package='gvido_vision',
        executable='stereo_filter',
        name='tag_tf_stabilizer',
        parameters=[os.path.join(pkg_share, 'config', 'tag_filter.yaml')],
        output='screen',
    )
    
    return LaunchDescription([
        oak_ns_arg,
        camera_frame_arg,
        oak_camera,
        static_tf,
        apriltag_delayed,
        tag_filter,
    ])