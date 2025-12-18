from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    rviz_config = get_package_share_directory('gvido_vision') + '/rviz/uniree_lidar.rviz'
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Запускать RViz (true/false)'
    )

    uniree_lidar_driver = Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters= [
                    
                    {'initialize_type': 2},
                    {'work_mode': 0},
                    {'use_system_timestamp': True},
                    {'range_min': 0.0},
                    {'range_max': 100.0},
                    {'cloud_scan_num': 18},

                    {'lidar_port': 6101},
                    {'lidar_ip': '192.168.1.62'},
                    {'local_port': 6201},
                    {'local_ip': '192.168.1.2'},
                    
                    {'cloud_frame': "unilidar_lidar"},
                    {'cloud_topic': "unilidar/cloud"},
                    {'imu_frame': "unilidar_imu"},
                    {'imu_topic': "unilidar/imu"},
                    ]
        )
    
    rviz_node = Node(
        namespace='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        rviz_arg,
        uniree_lidar_driver,
        rviz_node
    ])
