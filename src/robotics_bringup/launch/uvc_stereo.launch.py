from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"left_device": "/dev/v4l/by-id/LEFT_OV9281_UNCONFIGURED",
                "right_device": "/dev/v4l/by-id/RIGHT_OV9281_UNCONFIGURED",
                "calibration_file": "", "width": "640", "height": "480", "fps": "20.0"}
    params = {k: LaunchConfiguration(k) for k in defaults}
    for key in ("width", "height"):
        params[key] = ParameterValue(LaunchConfiguration(key), value_type=int)
    params["fps"] = ParameterValue(LaunchConfiguration("fps"), value_type=float)
    camera = Node(package="stereo_perception", executable="uvc_stereo_node", parameters=[params], output="screen")
    pipeline = Node(package="stereo_perception", executable="perception_node", parameters=[{
        "mode": "external", "calibration_file": LaunchConfiguration("calibration_file")
    }], output="screen")
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [
        camera, pipeline,
        RegisterEventHandler(OnProcessExit(target_action=camera, on_exit=[Shutdown(reason="camera unavailable")])),
        RegisterEventHandler(OnProcessExit(target_action=pipeline, on_exit=[Shutdown(reason="perception unavailable")]))])
