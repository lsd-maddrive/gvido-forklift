from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='oak'),
        DeclareLaunchArgument('namespace', default_value=''),
        
        ComposableNodeContainer(
            name='oak_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='depthai_ros_driver::Camera',
                    name='oak',
                    namespace='',
                    parameters=[{
                        'rgb.enable': True,
                        'stereo.enable': False,
                        'nn.enable': False,
                    }],
                )
            ],
            output='screen',
        )
    ])