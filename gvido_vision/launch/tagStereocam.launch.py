#!/usr/bin/env python3
# launch/tagStereocam.launch.py
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Динамический путь к калибровочному файлу (если есть)
    pkg_vision_share = get_package_share_directory('gvido_vision')
    default_calib_path = os.path.join(pkg_vision_share, 'config', 'calibration', 'lcalib.yaml')
    
    # Проверяем, существует ли файл, если нет — используем пустую строку
    calib_url = f'file://{default_calib_path}' if os.path.exists(default_calib_path) else ''

    # ── Аргументы для левой камеры (USB) ───────────────────────────────────────
    camera_name_L_arg = DeclareLaunchArgument(
        'camera_name_L',
        default_value='camera_left',
        description='Namespace / имя левой камеры'
    )
    video_device_L_arg = DeclareLaunchArgument(
        'video_device_L',
        default_value='/dev/video0',
        description='Путь к устройству левой камеры'
    )
    camera_info_url_L_arg = DeclareLaunchArgument(
        'camera_info_url_L',
        default_value=calib_url,
        description='URL калибровки левой камеры'
    )

    # ── Аргументы для правой камеры (OAK-D) ────────────────────────────────────
    camera_name_R_arg = DeclareLaunchArgument(
        'camera_name_R',
        default_value='camera_right',
        description='Namespace / имя правой камеры'
    )
    oak_ns_arg = DeclareLaunchArgument(
        'oak_ns',
        default_value='oak',
        description='Namespace для OAK-D камеры'
    )

    # ── Общие аргументы ────────────────────────────────────────────────────────
    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='mjpeg2rgb',
        description='Формат: mjpeg2rgb, yuyv, raw_mjpeg'
    )
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Ширина изображения'
    )
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Высота изображения'
    )
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Частота кадров'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='image_raw',
        description='Топик изображения для AprilTag'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='tags_36h11_multi.yaml',
        description='Файл конфигурации AprilTag'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Использовать симуляционное время'
    )
    intra_process_arg = DeclareLaunchArgument(
        'intra_process',
        default_value='true',
        description='Включить внутрипроцессное общение'
    )
    multithread_arg = DeclareLaunchArgument(
        'multithread',
        default_value='true',
        description='Использовать многопоточность в контейнере'
    )

    # ── Узел левой камеры (USB) ────────────────────────────────────────────────
    camera_node_L = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=LaunchConfiguration('camera_name_L'),
        namespace=LaunchConfiguration('camera_name_L'),
        parameters=[{
            'video_device': LaunchConfiguration('video_device_L'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'camera_name': LaunchConfiguration('camera_name_L'),
            'camera_info_url': LaunchConfiguration('camera_info_url_L'),
            'frame_id': 'camera_link_L',
            'io_method': 'mmap',
            'brightness': -1,
            'contrast': -1,
            'saturation': -1,
            'sharpness': -1,
            'gain': -1,
            'auto_white_balance': True,
        }],
        output='screen',
    )

    # ── Узел правой камеры (OAK-D) ─────────────────────────────────────────────
# ── Узел правой камеры (OAK-D) ─────────────────────────────────────────────
    oak_camera_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        namespace=LaunchConfiguration('oak_ns'),
        parameters=[{
            '/home/arya/ros2_ws/src/gvido-forklift/gvido_vision/config/oak_camera.yaml'
        }],
        output='screen',
    )

    # ── Статическая трансформация между камерами ────────────────────────────────
    static_tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_cameras',
        arguments=['0.4654', '0.0095', '0.0096', '0.0441', '0.0493', '-0.0213', 'camera_link_L', 'camera_link_R'],
        output='screen'
    )
    
    static_tf_baseToCam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_cameras',
        arguments=['0.0000', '0.2328', '1.2450', '-1.5917', '0.0201', '-1.5821', 'base_link', 'camera_link_L'],
        output='screen'
    )

    # ── AprilTag (multi) ───────────────────────────────────────────────────────
    # ИСПРАВЛЕНО: используем просто namespace + топик
    apriltag_multi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('apriltag_ros'),
                'launch',
                'multi_tag_detector.launch.py'
            ])
        ),
        launch_arguments={
            'camera_name_L': LaunchConfiguration('camera_name_L'),
            'camera_name_R': PathJoinSubstitution([
                # топик oak/camera/right/image_raw
                LaunchConfiguration('oak_ns'),
                'camera',
                'rgb'
            ]),
            'image_topic': LaunchConfiguration('image_topic'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'intra_process': LaunchConfiguration('intra_process'),
            'multithread': LaunchConfiguration('multithread'),
            'container_name': 'tag_container_multi',
        }.items()
    )
    print(PathJoinSubstitution([LaunchConfiguration('oak_ns'),'rgb']))

    # apriltag_multi = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('apriltag_ros'),
    #             'launch',
    #             'multi_tag_detector.launch.py'
    #         ])
    #     ),
    #     launch_arguments={
    #         'camera_name_L': LaunchConfiguration('camera_name_L'),
    #         'camera_name_R': 'oak/rgb',
    #         'image_topic': LaunchConfiguration('image_topic'),
    #         'params_file': LaunchConfiguration('params_file'),
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'intra_process': LaunchConfiguration('intra_process'),
    #         'multithread': LaunchConfiguration('multithread'),
    #         'container_name': 'tag_container_multi',
    #     }.items()
    # )

    # Фильтр для усреднения тегов
    tag_filter = Node(
        package='gvido_vision',
        executable='stereo_filter',
        name='tag_tf_stabilizer',
        parameters=[os.path.join(get_package_share_directory('gvido_vision'), 'config', 'tag_filter.yaml')],
        output='screen',
    )

    # Статическая трансформация для frame_id OAK-D
    static_tf_oak = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_oak',
        arguments=['0.0000', '0.2328', '1.2450', '-1.5917', '0.0201', '-1.5821', 
                'base_link', 'camera_rgb_camera_optical_frame'],
        output='screen',
    )



    return LaunchDescription ([
        # Аргументы
        camera_name_L_arg,
        video_device_L_arg,
        camera_info_url_L_arg,
        camera_name_R_arg,
        oak_ns_arg,
        pixel_format_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        image_topic_arg,
        params_file_arg,
        use_sim_time_arg,
        intra_process_arg,
        multithread_arg,

        # Узлы камер
        camera_node_L,
        oak_camera_node,
        
        # Статическая трансформация
        static_tf_cam,
        static_tf_baseToCam,
        static_tf_oak,

        # AprilTag детекторы
        apriltag_multi,

        # Фильтр тэгов
        tag_filter,
    ])