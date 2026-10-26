"""Hardware-inert mission status node; action execution is added after planner integration."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .state_machine import MissionMachine

class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("hardware_output_enabled", False)
        if self.get_parameter("hardware_output_enabled").value:
            raise RuntimeError("mission manager cannot enable physical hardware output")
        self.machine = MissionMachine()
        self.publisher = self.create_publisher(String, "mission/status", 10)
        self.create_timer(1.0, self.publish_status)
    def publish_status(self) -> None:
        msg = String(); msg.data = f"{self.machine.state.value}:{self.machine.detail}"; self.publisher.publish(msg)
def main(args=None) -> None:
    rclpy.init(args=args); node = MissionManager()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
