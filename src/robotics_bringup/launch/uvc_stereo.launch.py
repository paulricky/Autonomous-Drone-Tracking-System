from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = LaunchConfiguration("config")
    default = str(Path(get_package_share_directory("robotics_bringup")) / "config" / "hardware.yaml")
    camera = Node(package="stereo_perception", executable="uvc_stereo_node", parameters=[config], output="screen")
    pipeline = Node(package="stereo_perception", executable="perception_node", parameters=[config], output="screen")
    return LaunchDescription([DeclareLaunchArgument("config", default_value=default),
        camera, pipeline,
        RegisterEventHandler(OnProcessExit(target_action=camera, on_exit=[Shutdown(reason="camera unavailable")])),
        RegisterEventHandler(OnProcessExit(target_action=pipeline, on_exit=[Shutdown(reason="perception unavailable")]))])
