from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    executable = str(Path(get_package_prefix("stereo_perception")) / "lib" /
                     "stereo_perception" / "calibrate_stereo")
    defaults = {
        "left_glob": "calibration_data/left_*.png",
        "right_glob": "calibration_data/right_*.png",
        "columns": "0", "rows": "0", "square_m": "0",
        "output": "calibration_data/stereo.yaml",
        "left_camera_id": "UNCONFIGURED", "right_camera_id": "UNCONFIGURED",
    }
    args = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    args.append(ExecuteProcess(cmd=[executable] + [LaunchConfiguration(key) for key in defaults],
                               output="screen"))
    return LaunchDescription(args)
