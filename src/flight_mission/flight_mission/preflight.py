"""Read-only live readiness report. Never arms, disarms, lands or shuts down."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from flight_interfaces.msg import FlightControllerStatus, MspRawTelemetry, VehicleState
from .readiness import evaluate


class PreflightNode(Node):
    def __init__(self):
        super().__init__("flight_readiness")
        self.declare_parameter("operator_authorized", False)
        if self.declare_parameter("hardware_output_enabled", False).value:
            raise RuntimeError("preflight cannot enable physical output")
        self.controller = self.raw = self.state = None
        self.create_subscription(FlightControllerStatus, "flight_controller/status", self.on_controller, 10)
        self.create_subscription(MspRawTelemetry, "flight_controller/raw_telemetry", self.on_raw, 10)
        self.create_subscription(VehicleState, "perception/state_estimate", self.on_state, qos_profile_sensor_data)
        self.publisher = self.create_publisher(String, "preflight/status", 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_service(Trigger, "preflight/check", self.check)
        self.create_timer(1.0, self.publish)

    def on_controller(self, msg): self.controller = msg
    def on_raw(self, msg): self.raw = msg
    def on_state(self, msg): self.state = msg

    def fresh(self, stamp, max_age):
        age = self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
        return math.isfinite(age) and 0 <= age <= max_age

    def report(self):
        c, r, s = self.controller, self.raw, self.state
        # 1 Hz read-only inventory is diagnostic only. A separate 0.25 s
        # physical command gate remains closed; this check does not relax it.
        telemetry = bool(c and r and c.connected and r.status_available and
                         r.i2c_errors == 0 and self.fresh(c.stamp, 0.25) and self.fresh(r.poll_started, 0.25))
        state = bool(s and s.valid and not s.ground_truth and s.header.frame_id == "map" and
                     self.fresh(s.header.stamp, .25) and all(math.isfinite(v) for v in
                     (s.position.x, s.position.y, s.position.z, s.velocity.x, s.velocity.y, s.velocity.z)))
        return evaluate(operator_authorized=self.get_parameter("operator_authorized").value,
                        telemetry_fresh=telemetry, position_velocity_fresh=state,
                        output_disabled=not c or not (c.hardware_output_enabled or c.command_gate_open))

    def check(self, _, response):
        report = self.report()
        response.success = report["flight_ready"]
        response.message = json.dumps(report)
        return response

    def publish(self):
        report = self.report()
        msg = String(); msg.data = json.dumps(report); self.publisher.publish(msg)
        array = DiagnosticArray(); array.header.stamp = self.get_clock().now().to_msg()
        for check in report["checks"]:
            status = DiagnosticStatus()
            status.name = "preflight/" + check["condition"]
            status.hardware_id = "unverified-aircraft"
            status.level = {"PASS": DiagnosticStatus.OK, "FAIL": DiagnosticStatus.ERROR,
                            "UNVERIFIED": DiagnosticStatus.STALE}[check["result"]]
            status.message = check["result"] + ": " + check["reason"]
            status.values = [KeyValue(key="corrective_action", value=check["corrective_action"])]
            array.status.append(status)
        self.diagnostics.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = PreflightNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
