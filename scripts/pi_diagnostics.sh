#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
output="${1:-${ROBOTICS_DATA_DIR:-deployment_data}/diagnostics/$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$output"
# A missing/disconnected device or hung utility must not prevent the rest of
# the evidence from being collected. Ubuntu coreutils provides timeout.
capture() {
  local file="$1" status
  shift
  if timeout 15s "$@" >> "$output/$file" 2>&1; then
    return 0
  else
    status=$?
    printf 'diagnostic command exited %s: %s\n' "$status" "$*" >> "$output/$file"
  fi
}
capture system.txt uname -a
capture system.txt cat /etc/os-release
capture usb.txt lsusb
capture cameras.txt v4l2-ctl --list-devices
capture device_paths.txt ls -l /dev/v4l/by-id /dev/v4l/by-path /dev/serial/by-id /dev/serial/by-path
if command -v v4l2-ctl >/dev/null 2>&1; then
  for device in /dev/v4l/by-id/* /dev/v4l/by-path/*; do
    [[ -e "$device" ]] || continue
    {
      printf 'device=%s target=%s\n' "$device" "$(readlink -f "$device")"
      capture video_modes.txt v4l2-ctl -d "$device" --all
      capture video_modes.txt v4l2-ctl -d "$device" --list-formats-ext
      if command -v udevadm >/dev/null 2>&1; then
        capture video_modes.txt udevadm info --query=property --name="$device"
      fi
    } >> "$output/video_modes.txt" 2>&1
  done
fi
for device in /dev/serial/by-id/* /dev/serial/by-path/*; do
  [[ -e "$device" ]] || continue
  {
    printf 'device=%s target=%s\n' "$device" "$(readlink -f "$device")"
    capture serial_devices.txt stat -Lc 'mode=%A owner=%U group=%G' "$device"
    if command -v udevadm >/dev/null 2>&1; then
      capture serial_devices.txt udevadm info --query=property --name="$device"
    fi
  } >> "$output/serial_devices.txt" 2>&1
done
capture container.log docker logs "${ROBOTICS_CONTAINER:-robotics-stack}"
capture ros_graph.txt docker exec "${ROBOTICS_CONTAINER:-robotics-stack}" bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /opt/robotics/install/setup.bash && { timeout 5s ros2 node list; timeout 5s ros2 topic list -t; }'
echo "$output"
