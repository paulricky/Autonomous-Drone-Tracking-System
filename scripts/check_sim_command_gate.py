#!/usr/bin/env python3
"""Exercise the software-only simulator command input without flight hardware."""
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data

from flight_interfaces.msg import ControlCommand, VehicleState


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("sim_command_gate_check")
    states = []
    subscription = node.create_subscription(
        VehicleState, "state/ground_truth",
        lambda message: states.append((message.position.x, message.velocity.x)),
        qos_profile_sensor_data,
    )
    publisher = node.create_publisher(
        ControlCommand, "control/acceleration", qos_profile_sensor_data
    )
    process = subprocess.Popen(
        ["ros2", "run", "robotics_core", "autonomy_sim_node", "--ros-args",
         "-p", "auto_start:=false", "-p", "scenario:=free"],
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 8
        while len(states) < 5 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(states) >= 5, "simulated state did not start"

        def send_for(duration: float, acceleration: float, *, stale: bool = False) -> None:
            end = time.monotonic() + duration
            while time.monotonic() < end:
                command = ControlCommand()
                command.header.frame_id = "map"
                command.header.stamp = node.get_clock().now().to_msg()
                if stale:
                    command.header.stamp.sec -= 1
                command.trajectory_id = 1
                command.acceleration.x = acceleration
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.02)

        send_for(0.25, float("nan"))
        assert process.poll() is None, "invalid command crashed the simulator"
        assert all(math.isfinite(x) and math.isfinite(v) for x, v in states)
        assert abs(states[-1][0]) < 0.01, "invalid command moved the vehicle"

        send_for(0.5, 1.0)
        assert states[-1][0] > 0.03, "valid command did not reach the simulator"
        speed_before_replay = states[-1][1]
        send_for(0.45, 2.0, stale=True)
        assert process.poll() is None, "stale command crashed the simulator"
        assert states[-1][1] < speed_before_replay, "stale command was applied"
        assert all(math.isfinite(x) and math.isfinite(v) for x, v in states)
        print("simulation command gate passed")
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
