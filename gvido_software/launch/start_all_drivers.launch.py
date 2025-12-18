from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():

    launch_arguments = [
        # Параметры для включения/выключения драйверов
        DeclareLaunchArgument(
            "launch_lidar_driver",
            default_value="true",
            description="Запускать драйвер лидара"
        ),
        DeclareLaunchArgument(
            "launch_camera_driver",
            default_value="true",
            description="Запускать драйвер камеры"
        ),
        DeclareLaunchArgument(
            "launch_imu_driver",
            default_value="true",
            description="Запускать драйвер IMU"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

    ]

    
    # --- Запуск Tf Transform ---
    tf_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gvido_software'),
                'launch',
                'tf_transform.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    oak_d_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gvido_vision'),
                'launch/drivers',
                'oak_d_pro_camera.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_camera_driver'))
    )


    uniree_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gvido_vision'),
                'launch/drivers',
                'unitree_lidar_driver.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_lidar_driver'))
    )
    
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='gvido_xbox_node',
            output='screen',
        )

    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_waveshare_node',
        condition=IfCondition(LaunchConfiguration('launch_imu_driver'))
    )  

    delayed_imu_node = TimerAction(
        period=3.0,
        actions=[imu_node]
    )

    all_actions = launch_arguments + [
        tf_transform_launch,
        # oak_d_camera_launch,
        uniree_lidar_launch,
        # joy_node,
        delayed_imu_node, 
    ]

    return LaunchDescription(all_actions)