"""Pi-local LAND interlocks and explicit readiness reporting; no actuator backend."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="flight_mission", executable="mission_manager",
             parameters=[{"hardware_output_enabled": False, "submit_navigation_goal": False,
                          "shutdown_authorized": False}], output="screen"),
        Node(package="flight_mission", executable="preflight_status",
             parameters=[{"hardware_output_enabled": False}], output="screen"),
    ])
