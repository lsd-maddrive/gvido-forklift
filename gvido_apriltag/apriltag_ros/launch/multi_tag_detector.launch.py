#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def _create_container(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    if not os.path.isabs(params_file):
        params_file = os.path.join(
            get_package_share_directory("apriltag_ros"), "config", params_file)

    if not Path(params_file).exists():
        raise RuntimeError(f"Parameters file '{params_file}' does not exist")

    multi_node = ComposableNode(
        package="apriltag_ros",
        plugin="apriltag_ros::MultiAprilTagNode",
        name="multi_apriltag",
        parameters=[
            params_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "left_camera_name": LaunchConfiguration("camera_name_L"),
                "right_camera_name": LaunchConfiguration("camera_name_R"),
                "image_topic": LaunchConfiguration("image_topic"),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("intra_process")}],
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name").perform(context),
        namespace="apriltag",
        package="rclcpp_components",
        executable="component_container_mt" if LaunchConfiguration("multithread").perform(context).lower() == "true" else "component_container",
        composable_node_descriptions=[multi_node],
        output="screen",
    )
    return [container]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument("camera_name_L", default_value="camera_left"),
        DeclareLaunchArgument("camera_name_R", default_value="camera_right"),
        DeclareLaunchArgument("image_topic", default_value="image_raw"),
        DeclareLaunchArgument("params_file", default_value="tags_36h11_multi.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("intra_process", default_value="true"),
        DeclareLaunchArgument("multithread", default_value="true"),
        DeclareLaunchArgument("container_name", default_value="tag_container_multi"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    env_unbuffered = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    container_creator = OpaqueFunction(function=_create_container)

    return LaunchDescription(launch_args + [env_unbuffered, container_creator])