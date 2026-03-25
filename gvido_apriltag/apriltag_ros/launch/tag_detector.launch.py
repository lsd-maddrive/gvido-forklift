#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _create_container(context):
    # --- Чтение launch‑аргументов ---
    camera_name  = LaunchConfiguration("camera_name").perform (context)
    image_topic  = LaunchConfiguration("image_topic").perform(context)
    params_file  = LaunchConfiguration("params_file").perform(context)
    multithread  = LaunchConfiguration("multithread").perform(context).lower() == "true"
    container_name = LaunchConfiguration("container_name").perform(context)

    # --- Валидируем и нормализуем путь к YAML‑файлу ---
    if not os.path.isabs(params_file):
        params_file = os.path.join(
            get_package_share_directory("apriltag_ros"),
            "config", params_file,
        )
    if not Path(params_file).exists():
        raise RuntimeError(f"Parameters file '{params_file}' does not exist")

    # --- Описание composable‑узла ---
    tag_node = ComposableNode(
        package="apriltag_ros",
        plugin="apriltag_ros::AprilTagNode",
        name="apriltag",
        parameters=[
            params_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("image",        f"{camera_name}/{image_topic}"),
            ("camera_info",  f"{camera_name}/camera_info"),
        ],
        extra_arguments=[
            {"use_intra_process_comms": LaunchConfiguration("intra_process")},
        ],
    )

    executable = "component_container_mt" if multithread else "component_container"
    container = ComposableNodeContainer(
        name=container_name,
        namespace="apriltag",
        package="rclcpp_components",
        executable=executable,
        composable_node_descriptions=[tag_node],
        output="screen",
    )
    return [container]


def generate_launch_description():

    # --- Launch‑аргументы с умолчаниями ---
    launch_args = [
        DeclareLaunchArgument(
            "camera_name", default_value="right",
            description="Namespace префикс камеры (например, 'right')"),
        DeclareLaunchArgument(
            "image_topic", default_value="image_rect",
            description="Относительный топик изображения"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("apriltag_ros"),
                "config", "tags_36h11_multi.yaml"),
            description="Путь к YAML с параметрами априлтега"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Использовать /clock (симуляция)"),
        DeclareLaunchArgument(
            "intra_process", default_value="true",
            description="Включить ROS 2 intra‑process коммуникацию"),
        DeclareLaunchArgument(
            "multithread", default_value="true",
            description="Использовать многопоточный контейнер"),
        DeclareLaunchArgument(
            "container_name", default_value="tag_container",
            description="Имя контейнера с компонентом"),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="Уровень логирования RCUTILS"),
    ]

    env_unbuffered = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # --- OpaqueFunction для создания контейнера после подстановки конфигураций ---
    container_creator = OpaqueFunction(function=_create_container)
    return LaunchDescription(launch_args + [env_unbuffered, container_creator])
 