#!/usr/bin/env python3
"""ROS 2 smoke client for NavigateToPose acceptance and cancellation."""
import sys,time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ece3162_interfaces.action import NavigateToPose

def wait(node,future,timeout):
    end=time.monotonic()+timeout
    while rclpy.ok() and not future.done() and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
    return future.done()

def main():
    rclpy.init();node=Node("navigation_action_smoke_client")
    client=ActionClient(node,NavigateToPose,"navigate_to_pose")
    if not client.wait_for_server(timeout_sec=8):return 2
    goal=NavigateToPose.Goal();goal.goal.header.frame_id="map";goal.goal.pose.position.x=5.;goal.goal.pose.position.z=1.;goal.position_tolerance=.15
    sent=client.send_goal_async(goal)
    if not wait(node,sent,3) or not sent.result().accepted:return 3
    deadline=time.monotonic()+.5
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    cancelled=sent.result().cancel_goal_async()
    if not wait(node,cancelled,3):return 4
    responses=cancelled.result().goals_canceling
    print(f"action_accepted=true cancel_responses={len(responses)}")
    node.destroy_node();rclpy.shutdown();return 0 if responses else 5
if __name__=="__main__":sys.exit(main())
