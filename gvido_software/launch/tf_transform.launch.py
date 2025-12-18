from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

use_sim_time_arg = DeclareLaunchArgument(
    "use_sim_time",
    default_value="false",
    description="Использовать /clock из симуляции или bag-файла",
)

use_sim_time = LaunchConfiguration("use_sim_time")

# ──────────────────────────────────────────────────────────────
#                          Узлы TF
# ──────────────────────────────────────────────────────────────

static_tf_base_link_unilidar_lidar = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0.000000', '-0.148027', '1.510000', '-0.692301', '0.034416', '-0.719876', '0.036253',
        'base_link', 'unilidar_imu_initial'    ],
    parameters=[{"use_sim_time": use_sim_time}]
)

static_tf_base_link_waveshare= Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0.0', '0.0', '1.1', '0.0', '0.0', '0.0', 'base_link', 'base_imu_link'],
    output='screen',
    parameters=[{"use_sim_time": use_sim_time}]
)

static_tf_rslidar_right_oak_camera = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0.0', '0.0', '1.51', '0.0', '0.0', '0.0', 'base_link', 'oak-d-base-frame'],
    output='screen',
    parameters=[{"use_sim_time": use_sim_time}]
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        use_sim_time_arg, 
        static_tf_base_link_unilidar_lidar,
        static_tf_base_link_waveshare,
        static_tf_rslidar_right_oak_camera,
    ])