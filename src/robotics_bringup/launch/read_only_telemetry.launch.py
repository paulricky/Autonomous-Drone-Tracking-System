from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robotics_bringup.hardware_config import validate


def generate_launch_description():
    default = str(Path(get_package_share_directory("robotics_bringup")) / "config" / "hardware.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default),
        OpaqueFunction(function=lambda context: validate(context, telemetry=True)),
        Node(package="robotics_core", executable="msp_telemetry_node",
             parameters=[LaunchConfiguration("config")], output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(default).parents[1] / "launch/mission_safety.launch.py"))),
    ])
