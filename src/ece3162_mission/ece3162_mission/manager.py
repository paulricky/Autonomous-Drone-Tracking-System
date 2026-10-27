"""Low-rate mission readiness and action orchestration; no perception algorithms."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Empty
from sensor_msgs.msg import PointCloud2
from ece3162_interfaces.msg import VehicleState
from ece3162_interfaces.action import NavigateToPose
from .state_machine import MissionMachine

class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("hardware_output_enabled", False)
        if self.get_parameter("hardware_output_enabled").value:
            raise RuntimeError("mission manager cannot enable physical hardware output")
        self.machine = MissionMachine()
        self.declare_parameter("submit_navigation_goal", False)
        self.declare_parameter("goal_x", 4.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.0)
        self.executing = self.get_parameter("submit_navigation_goal").value
        self.publisher = self.create_publisher(String, "mission/manager_status" if self.executing else "mission/status", 10)
        self.state_stamp = self.map_stamp = 0.0
        self.state_valid = self.sent = False
        self.goal_handle = None
        if self.executing:
            self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.create_subscription(VehicleState, "perception/state_estimate", self.on_state, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, "mapping/occupied_voxels", self.on_map, qos_profile_sensor_data)
            self.create_subscription(Empty, "mission/cancel", self.cancel, 10)
        self.create_timer(0.1, self.tick)

    def on_state(self, message):
        self.state_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.state_valid = message.valid and not message.ground_truth and message.header.frame_id == "map"

    def on_map(self, message):
        if message.header.frame_id == "map":
            self.map_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def tick(self):
        self.publish_status()
        if not self.executing or self.sent:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if not (self.state_valid and 0 <= now-self.state_stamp < .5 and 0 <= now-self.map_stamp < .5 and self.client.server_is_ready()):
            return
        if not self.machine.request(True, True):
            return
        self.sent = True
        goal = NavigateToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.orientation.w = 1.0
        for axis in ("x", "y", "z"):
            setattr(goal.goal.pose.position, axis, self.get_parameter("goal_" + axis).value)
        goal.position_tolerance = .15
        self.client.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.machine.fail("navigation goal rejected")
            return
        self.machine.plan_ready()
        self.goal_handle.get_result_async().add_done_callback(self.finished)

    def finished(self, future):
        result = future.result().result
        if result.success:
            self.machine.complete()
        else:
            self.machine.fail(result.detail)

    def cancel(self, _):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.machine.cancel()

    def publish_status(self) -> None:
        msg = String(); msg.data = f"{self.machine.state.value}:{self.machine.detail}"; self.publisher.publish(msg)
def main(args=None) -> None:
    rclpy.init(args=args); node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
