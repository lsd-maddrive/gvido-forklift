from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    launch_arguments = [
        DeclareLaunchArgument("lidar_topic",  default_value="/unilidar/cloud"),
        DeclareLaunchArgument("imu_topic",   default_value="/imu/data"),
        DeclareLaunchArgument("icp", default_value="true"),
        DeclareLaunchArgument("frame_id",       default_value="base_link"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("ekf_yaml", default_value=PathJoinSubstitution([get_package_share_directory('forklift_vision'), 'config', 'ekf.yaml']), description="Путь к конфигу EKF")

    ]
    
    # --- ICP одометрия ---
    icp_odometry_left= Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': LaunchConfiguration('frame_id'),
            'odom_frame_id': 'icp_odom',
            'publish_tf': True,
            'wait_imu_to_init': True,
            'guess_frame_id': "",
            'approx_sync': True,

            # --- Параметры ICP ---
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/VoxelSize': '0.1',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',

            # --- Параметры одометрии ---
            'Odom/ScanKeyFrameThr': '0.9',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
            'OdomF2M/BundleAdjustment': 'false',
            'Icp/CorrespondenceRatio': '0.07'
        }],
        remappings=[
            ('scan_cloud', LaunchConfiguration('lidar_topic')),
            ('imu', LaunchConfiguration('imu_topic')),
            ('odom', '/icp_odom')

        ],
        condition=IfCondition(LaunchConfiguration('icp'))
    )


    # # --- EKF  ---
    # ekf_filter_node_odom = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node_odom',
    #     output='screen',
    #     parameters=[
    #         LaunchConfiguration('ekf_yaml'),
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ]
    # )


    all_actions = launch_arguments + [
        icp_odometry_left,
        # ekf_filter_node_odom,
    ]

    return LaunchDescription(all_actions)