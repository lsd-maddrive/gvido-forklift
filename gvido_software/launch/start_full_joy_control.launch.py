from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='gvido_xbox_node',
            output='screen',
        ),

        Node(
            package='gvido_bringup',
            executable='gvido_can_bringup',
            name='gvido_can_bringup',
            output='screen',
        ),
    ])
   