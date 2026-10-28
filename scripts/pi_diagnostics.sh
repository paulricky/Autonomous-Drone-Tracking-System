#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
output="${1:-deployment_data/diagnostics/$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$output"
uname -a > "$output/system.txt"
cat /etc/os-release >> "$output/system.txt"
lsusb > "$output/usb.txt" 2>&1 || true
v4l2-ctl --list-devices > "$output/cameras.txt" 2>&1 || true
ls -l /dev/v4l/by-id /dev/v4l/by-path /dev/serial/by-id > "$output/device_paths.txt" 2>&1 || true
docker logs "${ROBOTICS_CONTAINER:-robotics-stack}" > "$output/container.log" 2>&1 || true
docker exec "${ROBOTICS_CONTAINER:-robotics-stack}" bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /opt/robotics/install/setup.bash; ros2 node list; ros2 topic list -t' \
  > "$output/ros_graph.txt" 2>&1 || true
echo "$output"
