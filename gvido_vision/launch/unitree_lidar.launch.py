#!/usr/bin/env python3
"""
Launch-файл для запуска лидара Unitree L2
+ статические трансформации base_link -> unilidar_lidar
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # 1. Официальный launch драйвера лидара Unitree
    # ============================================================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('unitree_lidar_ros2'),
                'launch.py'
            )
        ),
    )

    # ============================================================
    # 2. Статическая трансформация: map -> base_link
    # ============================================================
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen',
    )

    # ============================================================
    # 3. Статическая трансформация: base_link -> unilidar_lidar
    # ============================================================
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '1.8', '0', '-0.1', '0', 'base_link', 'unilidar_lidar'],
        output='screen',
    )

    return LaunchDescription([
        lidar_launch,
        static_tf_map_to_base,
        static_tf_base_to_lidar,
    ])