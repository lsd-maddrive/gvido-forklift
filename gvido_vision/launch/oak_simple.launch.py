from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'rgb.enable': True,
                'stereo.enable': False,
                'nn.enable': False,
            }],
            remappings=[
                ('/rgb/image_raw', '/camera_right/image_raw'),
            ]
        )
    ])
