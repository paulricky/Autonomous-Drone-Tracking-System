#!/usr/bin/env python3
"""Start the native onboard process supervisor. Startup never arms or takes off."""
import argparse
import os
from pathlib import Path
import platform
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=os.environ.get('DRONE_CONFIG', '/var/lib/robotics/config/physical_flight.yaml'))
    args = parser.parse_args()
    if platform.system() != 'Linux':
        parser.error('physical onboard application requires Linux on the Raspberry Pi')
    config = Path(args.config).resolve()
    if not config.is_file():
        parser.error('missing measured hardware configuration: '+str(config))
    if not shutil.which('ros2'):
        parser.error('source /opt/ros/jazzy/setup.bash and the built workspace install/setup.bash')
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    os.execvp('ros2', ['ros2', 'run', 'robotics_core', 'drone_supervisor_node',
                      '--ros-args', '-p', 'config_file:='+str(config)])


if __name__ == '__main__':
    main()
