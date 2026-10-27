from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
def generate_launch_description():
    params = str(Path(get_package_share_directory("robotics_bringup")) / "config" / "system.yaml")
    hardware = LifecycleNode(package="robotics_core", executable="hardware_monitor_node", name="hardware_monitor", namespace="",
                             parameters=[params], output="screen")
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(hardware),
                                            transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(target_lifecycle_node=hardware, goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(hardware),
                                             transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        hardware, activate, configure,
        Node(package="flight_mission", executable="mission_manager", name="mission_manager",
             parameters=[params], output="screen")])
