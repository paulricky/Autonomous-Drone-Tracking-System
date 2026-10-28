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
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(root / "config" / "hardware.yaml")),
        OpaqueFunction(function=lambda context: validate(context, cameras=True, telemetry=True, integrated=True)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root / "launch" / "uvc_stereo.launch.py")),
                                 launch_arguments={"config": config}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root / "launch" / "read_only_telemetry.launch.py")),
                                 launch_arguments={"config": config}.items()),
        Node(package="robotics_core", executable="autonomy_sim_node", parameters=[config],
             output="screen"),
        Node(package="robotics_core", executable="physical_command_gate_node",
             parameters=[config], output="screen"),
    ])
