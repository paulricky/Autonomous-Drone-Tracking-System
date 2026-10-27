"""Native stereo observations -> occupancy/PnP state -> native navigation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, EmitEvent
from launch.events import matches_action
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    navigation_parameters = str(Path(get_package_share_directory("robotics_bringup")) / "config" / "navigation.yaml")
    def shutdown_once(event, context):
        if not context.is_shutdown:
            return [Shutdown(reason="navigation or perception terminal state")]
        return []
    args = [DeclareLaunchArgument("results_directory", default_value="results/perception/navigation"),
            DeclareLaunchArgument("dynamic_after_s", default_value="-1.0"),
            DeclareLaunchArgument("disconnect_after_frames", default_value="-1"),
            DeclareLaunchArgument("low_texture_after_frames", default_value="-1"),
            DeclareLaunchArgument("static_obstacle", default_value="true")]
    perception = Node(package="stereo_perception", executable="perception_node", parameters=[{
        "mode": "synthetic", "frame_rate": 10.0,
        "results_directory": LaunchConfiguration("results_directory"),
        "dynamic_after_s": ParameterValue(LaunchConfiguration("dynamic_after_s"), value_type=float),
        "disconnect_after_frames": ParameterValue(LaunchConfiguration("disconnect_after_frames"), value_type=int),
        "low_texture_after_frames": ParameterValue(LaunchConfiguration("low_texture_after_frames"), value_type=int),
        "static_obstacle": ParameterValue(LaunchConfiguration("static_obstacle"), value_type=bool),
    }], output="screen")
    autonomy = Node(package="robotics_core", executable="autonomy_sim_node", parameters=[navigation_parameters, {
        "scenario": "free", "goal_x": 4.0, "state_topic": "perception/state_estimate",
        "auto_start": False,
        "use_perception_map": True, "unknown_is_free": True, "minimum_map_updates": 3,
        "hardware_output_enabled": False, "state_timeout": 0.5,
    }], output="screen")
    stop = RegisterEventHandler(OnProcessExit(target_action=autonomy, on_exit=shutdown_once))
    fail = RegisterEventHandler(OnProcessExit(target_action=perception, on_exit=shutdown_once))
    mission = Node(package="flight_mission", executable="mission_manager", parameters=[{
        "submit_navigation_goal": True, "goal_x": 4.0, "hardware_output_enabled": False}], output="screen")
    hardware = LifecycleNode(package="robotics_core", executable="hardware_monitor_node", name="hardware_monitor",
                             namespace="", parameters=[{"hardware_output_enabled": False}], output="screen")
    activate = RegisterEventHandler(OnStateTransition(target_lifecycle_node=hardware, goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(hardware), transition_id=Transition.TRANSITION_ACTIVATE))]))
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(hardware), transition_id=Transition.TRANSITION_CONFIGURE))
    return LaunchDescription(args + [hardware, activate, configure, perception, autonomy, mission, stop, fail])
