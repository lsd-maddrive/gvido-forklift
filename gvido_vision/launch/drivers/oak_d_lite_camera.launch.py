import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration("name").perform(context)
    namespace = LaunchConfiguration("namespace", default="").perform(context)

    # Получаем параметры из YAML файла для URDF запуска
    parent_frame = LaunchConfiguration("parent_frame", default="oak-d-base-frame").perform(context)
    camera_model = LaunchConfiguration("camera_model", default="OAK-D-LITE")
    use_composition = LaunchConfiguration("rsp_use_composition", default="true")
    rs_compat = LaunchConfiguration("rs_compat", default="false")

    # Получаем путь к depthai_descriptions для urdf_launch.py
    urdf_launch_dir = os.path.join(
        get_package_share_directory("depthai_descriptions"), "launch"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, "urdf_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "tf_prefix": name,
                "camera_model": camera_model,
                "base_frame": name,
                "parent_frame": parent_frame,
                "cam_pos_x": LaunchConfiguration("cam_pos_x", default="0.0"),
                "cam_pos_y": LaunchConfiguration("cam_pos_y", default="0.0"),
                "cam_pos_z": LaunchConfiguration("cam_pos_z", default="0.0"),
                "cam_roll": LaunchConfiguration("cam_roll", default="0.0"),
                "cam_pitch": LaunchConfiguration("cam_pitch", default="0.0"),
                "cam_yaw": LaunchConfiguration("cam_yaw", default="0.0"),
                "use_composition": use_composition,
                "use_base_descr": "false",
                "rs_compat": rs_compat,
            }.items(),
        ),
        ComposableNodeContainer(
            name=f"{name}_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    namespace=namespace,
                    parameters=[params_file],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="both",
        )
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("gvido_vision")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("rsp_use_composition", default_value="true"),
        DeclareLaunchArgument("rs_compat", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "camera_rgb_imu.yaml"),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )