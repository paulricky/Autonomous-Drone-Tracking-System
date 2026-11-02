#!/usr/bin/env python3
"""Launch the physical ROS application; refuse ordinary shutdown while airborne."""
import argparse
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=os.environ.get('DRONE_CONFIG', '/var/lib/robotics/config/physical_flight.yaml'))
    args = parser.parse_args()
    if platform.system() != 'Linux':
        parser.error('physical onboard application requires Linux on the Raspberry Pi')
    if not Path(args.config).is_file():
        parser.error('missing measured hardware configuration: '+args.config)
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    try:
        import rclpy
        from std_msgs.msg import String
    except ImportError:
        parser.error('source /opt/ros/jazzy/setup.bash and your colcon install/setup.bash first')
    rclpy.init()
    node = rclpy.create_node('drone_application_supervisor')
    state = {'receipt': 0., 'status': {}}
    def observe(msg):
        try:
            status = json.loads(msg.data)
            state.update(receipt=time.monotonic(), status=status)
        except ValueError:
            return
        if status != state.get('printed'):
            print('Physical flight status:', status, flush=True)
            state['printed'] = status
    node.create_subscription(String, 'flight/status', observe, 10)
    emergency = node.create_publisher(String, 'flight/operator_request', 1)
    child = subprocess.Popen(['ros2', 'launch', 'robotics_bringup', 'physical_flight.launch.py', 'config:='+str(Path(args.config).resolve())], start_new_session=True)
    stopping = [False]
    signal.signal(signal.SIGINT, lambda *_: stopping.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))
    print('Application started; hardware readiness pending. Startup never arms or takes off.', flush=True)
    try:
        while child.poll() is None:
            rclpy.spin_once(node, timeout_sec=.1)
            if stopping[0]:
                status = state['status']
                if time.monotonic()-state['receipt'] < .4 and status.get('connected') and status.get('identity_verified') and status.get('ground') and status.get('armed') is False:
                    os.killpg(child.pid, signal.SIGINT)
                    break
                emergency.publish(String(data='EMERGENCY'))
                if not state.get('warned'):
                    print('Shutdown pending: requesting emergency landing; waiting for controller ground and disarmed confirmation. Independent pilot recovery remains available.', flush=True)
                    state['warned'] = True
        return child.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
