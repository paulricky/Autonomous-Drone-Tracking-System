#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
image="${ROBOTICS_IMAGE:-robotics-jazzy:local}"
container="${ROBOTICS_CONTAINER:-robotics-stack}"
data="${ROBOTICS_DATA_DIR:-$PWD/deployment_data}"
prepare_data() {
  mkdir -p "$data/calibration" "$data/logs" "$data/config"
  if [[ ! -e "$data/config/hardware.yaml" ]]; then
    cp src/robotics_bringup/config/hardware.yaml "$data/config/hardware.yaml"
  fi
}
record_build() {
  prepare_data
  {
    echo "image=$image"
    docker image inspect --format 'image_id={{.Id}} architecture={{.Architecture}}' "$image"
    git rev-parse HEAD
    git status --short
  } > "$data/build-info.txt"
}
run_hardware() {
  local launch="$1"
  shift
  prepare_data
  local devices=()
  if [[ "$launch" == "uvc_stereo.launch.py" || "$launch" == "hardware_integration.launch.py" ]]; then
    : "${LEFT_CAMERA_DEVICE:?set LEFT_CAMERA_DEVICE to a persistent /dev/v4l/by-id or by-path entry}"
    : "${RIGHT_CAMERA_DEVICE:?set RIGHT_CAMERA_DEVICE to a persistent /dev/v4l/by-id or by-path entry}"
    [[ -e "$LEFT_CAMERA_DEVICE" && -e "$RIGHT_CAMERA_DEVICE" && "$LEFT_CAMERA_DEVICE" != "$RIGHT_CAMERA_DEVICE" ]]
    devices+=(--device "$LEFT_CAMERA_DEVICE:$LEFT_CAMERA_DEVICE")
    devices+=(--device "$RIGHT_CAMERA_DEVICE:$RIGHT_CAMERA_DEVICE")
  fi
  if [[ "$launch" == "read_only_telemetry.launch.py" ]]; then
    : "${FLIGHT_CONTROLLER_DEVICE:?set FLIGHT_CONTROLLER_DEVICE to a persistent /dev/serial/by-id entry}"
  fi
  if [[ -n "${FLIGHT_CONTROLLER_DEVICE:-}" ]]; then
    [[ -e "$FLIGHT_CONTROLLER_DEVICE" &&
       ( "$FLIGHT_CONTROLLER_DEVICE" == /dev/serial/by-id/* ||
         "$FLIGHT_CONTROLLER_DEVICE" == /dev/serial/by-path/* ) ]]
    devices+=(--device "$FLIGHT_CONTROLLER_DEVICE:$FLIGHT_CONTROLLER_DEVICE")
  fi
  docker run --rm -it --network host --restart no --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}" \
    "${devices[@]}" \
    --mount "type=bind,src=$data/calibration,dst=/var/lib/robotics/calibration" \
    --mount "type=bind,src=$data/logs,dst=/var/log/robotics" \
    --mount "type=bind,src=$data/config,dst=/var/lib/robotics/config,readonly" \
    "$image" bash -lc "source /opt/ros/jazzy/setup.bash && source /opt/robotics/install/setup.bash && ros2 launch robotics_bringup $launch config:=/var/lib/robotics/config/hardware.yaml" "$@"
}
case "${1:-}" in
  build)
    docker build --pull -t "$image" -f Dockerfile .
    record_build
    ;;
  start)
    prepare_data
    # Diagnostics only. Hardware transport and autonomous output stay disabled.
    docker run -d --name "$container" --network host --restart no --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}" \
      --mount "type=bind,src=$data/calibration,dst=/var/lib/robotics/calibration" \
      --mount "type=bind,src=$data/logs,dst=/var/log/robotics" \
      --mount "type=bind,src=$data/config,dst=/var/lib/robotics/config" \
      "$image"
    ;;
  stop)
    docker stop "$container"
    ;;
  logs)
    docker logs "$container"
    ;;
  shell)
    docker exec -it "$container" bash
    ;;
  camera)
    run_hardware uvc_stereo.launch.py
    ;;
  telemetry)
    run_hardware read_only_telemetry.launch.py
    ;;
  integration)
    run_hardware hardware_integration.launch.py
    ;;
  update)
    docker build --pull -t "$image" -f Dockerfile .
    record_build
    docker stop "$container"
    docker rm "$container"
    "$0" start
    ;;
  *)
    echo "usage: $0 {build|start|stop|logs|shell|camera|telemetry|integration|update}" >&2
    exit 2
    ;;
esac
