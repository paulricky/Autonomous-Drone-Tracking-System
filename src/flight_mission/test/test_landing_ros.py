"""Exercise the real ROS mission services and native navigation server, no devices."""
import os
import json
import signal
import subprocess
import time
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from flight_interfaces.action import NavigateToPose
from flight_interfaces.msg import ControlCommand


def test_land_latches_native_navigation_and_denies_disarm_shutdown(tmp_path):
    namespace = "/landing_test_" + str(os.getpid())
    commands = [
        ["ros2", "run", "flight_mission", "mission_manager", "--ros-args", "-r", "__ns:=" + namespace],
        ["ros2", "run", "robotics_core", "autonomy_sim_node", "--ros-args", "-r", "__ns:=" + namespace,
         "-p", "simulate_vehicle:=false", "-p", "system_mode:=hardware_read_only",
         "-p", "auto_start:=false", "-p", "use_perception_map:=true"],
        ["ros2", "run", "flight_mission", "preflight_status", "--ros-args", "-r", "__ns:=" + namespace],
    ]
    logs, processes = [], []
    rclpy.init()
    node = rclpy.create_node("landing_interface_test", namespace=namespace)
    def wait(future, timeout=15):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        assert future.done(), "ROS operation timed out:\n" + "\n".join(
            path.read_text() for path in tmp_path.glob("process_*.log"))
        return future.result()
    def trigger(name):
        client = node.create_client(Trigger, "mission/" + name)
        assert client.wait_for_service(timeout_sec=15)
        return wait(client.call_async(Trigger.Request()))
    try:
        for index, command in enumerate(commands):
            log = (tmp_path / f"process_{index}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=15)
        land = node.create_client(Trigger, "mission/land")
        assert land.wait_for_service(timeout_sec=15)
        preflight = node.create_client(Trigger, "preflight/check")
        assert preflight.wait_for_service(timeout_sec=15)
        response = wait(preflight.call_async(Trigger.Request()))
        report = json.loads(response.message)
        assert not response.success and report["status"] == "NOT READY"
        conditions = {check["condition"]: check["result"] for check in report["checks"]}
        assert conditions["controlled_landing"] == "UNVERIFIED"
        assert conditions["physical_output_inhibited"] == "PASS"
        assert conditions["read_only_telemetry"] == "FAIL"
        goal = NavigateToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.orientation.w = 1.
        goal.goal.pose.position.x = 2.
        goal.goal.pose.position.z = 1.
        handle = wait(client.send_goal_async(goal))
        assert handle.accepted
        result_future = handle.get_result_async()
        response = wait(land.call_async(Trigger.Request()))
        assert not response.success and "landing_failed" in response.message
        result = wait(result_future)
        assert not result.result.success
        assert "LAND" in result.result.detail or "cancel" in result.result.detail.lower()
        # Allow the latched inhibit to reach the independent action server.
        latched = []
        node.create_subscription(Bool, "mission/navigation_inhibited", lambda m: latched.append(m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        end = time.monotonic() + 5
        while not latched and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        assert latched and all(latched)
        assert not wait(client.send_goal_async(goal)).accepted
        assert not trigger("land").success
        assert not trigger("disarm").success
        assert not trigger("prepare_shutdown").success
        # New command publication stays inhibited after LAND.
        commands_seen = []
        node.create_subscription(ControlCommand, "control/acceleration", commands_seen.append,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        end = time.monotonic() + .3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert not commands_seen
        assert all(process.poll() is None for process in processes)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for log in logs: log.close()
