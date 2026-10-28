from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(Path(get_package_share_directory("robotics_bringup")) / "config" / "hardware.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default),
        Node(package="robotics_core", executable="msp_telemetry_node",
             parameters=[LaunchConfiguration("config")], output="screen"),
    ])
