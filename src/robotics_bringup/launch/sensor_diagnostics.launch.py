"""Observe physical stereo localization and raw MSP telemetry without navigation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robotics_bringup.hardware_config import validate


def generate_launch_description():
    root = Path(get_package_share_directory("robotics_bringup"))
    config = LaunchConfiguration("config")
    actions = [
        DeclareLaunchArgument("config", default_value=str(root / "config/hardware.yaml")),
        OpaqueFunction(function=lambda context: validate(context, cameras=True, telemetry=True)),
    ]
    for launch in ("uvc_stereo", "read_only_telemetry"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(root / "launch" / (launch + ".launch.py"))),
            launch_arguments={"config": config}.items()))
    actions.append(Node(package="robotics_core", executable="physical_command_gate_node",
                        parameters=[config], output="screen"))
    return LaunchDescription(actions)
