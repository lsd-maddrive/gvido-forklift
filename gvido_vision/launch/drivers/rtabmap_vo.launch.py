#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Использовать /clock из симуляции или bag-файла'
    )

    launch_stereo_driver = DeclareLaunchArgument(
        'launch_stereo_driver',
        default_value='true',
        description='Запускать драйвер DepthAI (stereo_inertial_node)'
    )

    enable_rviz_stereo = DeclareLaunchArgument(
        'stereo_rviz',
        default_value='false',
        description='Запускать RViz для DepthAI драйвера'
    )

    mono_resolution = DeclareLaunchArgument(
        'mono_resolution',
        default_value='400p',
        description='Разрешение монокамер (DepthAI)'
    )

    # ────────────────────────────────────────────────
    #               Параметры RTAB-Map
    # ────────────────────────────────────────────────

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link_vo',
        description='Базовый фрейм робота'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data/waveshare',
        description='Топик с данными IMU'
    )

    tag_detections_topic_arg = DeclareLaunchArgument(
        'tag_detections_topic',
        default_value='/apriltag_detections',
        description='Топик с детекциями AprilTag'
    )

    launch_apriltag_detection = DeclareLaunchArgument("launch_apriltag_detection", default_value="true", description="Запускать детекцию Apriltag")
    oak_camera_name = DeclareLaunchArgument("camera_name", default_value="/right")
    oak_camera_image = DeclareLaunchArgument("image_topic", default_value="image_rect")
    

    # ────────────────────────────────────────────────
    #                   Запуск нод
    # ────────────────────────────────────────────────

    # Драйвер DepthAI (стерео + IMU)
    stereo_inertial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('forklift_vision'),
                'launch/drivers/camera',
                'oak_d_pro_inertial_node.launch.py'
            ])
        ]),
        launch_arguments={
            'depth_aligned': 'false',
            'enableRviz': LaunchConfiguration('stereo_rviz'),
            'monoResolution': LaunchConfiguration('mono_resolution'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_stereo_driver'))
    )

    # --- Детекция Apriltag ---
    apriltag_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('apriltag_ros'),
                'launch',
                'tag_detector.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'image_topic': LaunchConfiguration('image_topic')
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_apriltag_detection'))
    )

    # Параметры, общие для RTAB-Map нод
    rtabmap_common_params = {
        'frame_id': LaunchConfiguration('frame_id'),
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': True,
        'subscribe_tags': True,
        'tag_linear_variance': 0.0004,
        'tag_angular_variance': 0.0004,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    # rgbd_sync
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[rtabmap_common_params],
        remappings=[
            ('rgb/image',     '/right/image_rect'),
            ('rgb/camera_info', '/right/camera_info'),
            ('depth/image',   '/stereo/depth'),
        ]
    )

    # IMU filter (Madgwick)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('imu/data_raw', LaunchConfiguration('imu_topic')),
        ]
    )

    # RGBD Odometry
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[
        rtabmap_common_params,
        {                                         
            'odom_frame_id': 'odom_vo',
        }
        ],
        remappings=[
            ('imu', LaunchConfiguration('imu_topic')),
            ('tag_detections', LaunchConfiguration('tag_detections_topic')),
            ('odom', '/odom_vo'),
        ]
    )

    # RTAB-Map SLAM
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
        rtabmap_common_params,
        {                                         
            'odom_frame_id': 'odom_vo',
            'map_frame_id': 'map_vo',
        }
        ],        remappings=[
            ('imu', LaunchConfiguration('imu_topic')),
            ('tag_detections', LaunchConfiguration('tag_detections_topic')),
            ('odom', '/odom_vo'),

        ],
        arguments=['--delete_db_on_start']   
    )

    return LaunchDescription([
        use_sim_time,
        launch_stereo_driver,
        enable_rviz_stereo,
        mono_resolution,
        frame_id_arg,
        imu_topic_arg,
        tag_detections_topic_arg,
        launch_apriltag_detection,
        oak_camera_name,
        oak_camera_image,

        stereo_inertial_launch,
        apriltag_detector,

        rgbd_sync_node,
        imu_filter_node,
        rgbd_odometry_node,
        rtabmap_node,
    ])