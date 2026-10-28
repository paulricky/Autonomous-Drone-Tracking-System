#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
image="${ROBOTICS_IMAGE:-robotics-jazzy:local}"
container="${ROBOTICS_CONTAINER:-robotics-stack}"
data="${ROBOTICS_DATA_DIR:-$PWD/deployment_data}"
prepare_data() {
  mkdir -p "$data/calibration" "$data/logs" "$data/config"
  data="$(cd "$data" && pwd)"
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
build_image() {
  local platform daemon_arch
  daemon_arch="$(docker info --format '{{.Architecture}}')"
  case "$(uname -m):$daemon_arch" in
    aarch64:aarch64|aarch64:arm64) platform=linux/arm64 ;;
    x86_64:x86_64|x86_64:amd64) platform=linux/amd64 ;;
    *) echo "native build requires matching host and Docker daemon architectures" >&2; return 2 ;;
  esac
  docker build --platform "$platform" --pull -t "$image" -f Dockerfile .
  record_build
}
run_hardware() {
  local launch="$1"
  shift
  prepare_data
  local devices=()
  if [[ "$launch" == "uvc_stereo.launch.py" || "$launch" == "hardware_integration.launch.py" || "$launch" == "sensor_diagnostics.launch.py" ]]; then
    : "${LEFT_CAMERA_DEVICE:?set LEFT_CAMERA_DEVICE to a persistent /dev/v4l/by-id or by-path entry}"
    : "${RIGHT_CAMERA_DEVICE:?set RIGHT_CAMERA_DEVICE to a persistent /dev/v4l/by-id or by-path entry}"
    [[ -e "$LEFT_CAMERA_DEVICE" && -e "$RIGHT_CAMERA_DEVICE" && "$LEFT_CAMERA_DEVICE" != "$RIGHT_CAMERA_DEVICE" ]]
    [[ "$(readlink -f "$LEFT_CAMERA_DEVICE")" != "$(readlink -f "$RIGHT_CAMERA_DEVICE")" ]]
    devices+=(--device "$LEFT_CAMERA_DEVICE:$LEFT_CAMERA_DEVICE")
    devices+=(--device "$RIGHT_CAMERA_DEVICE:$RIGHT_CAMERA_DEVICE")
  fi
  if [[ "$launch" == "read_only_telemetry.launch.py" || "$launch" == "hardware_integration.launch.py" || "$launch" == "sensor_diagnostics.launch.py" ]]; then
    : "${FLIGHT_CONTROLLER_DEVICE:?set FLIGHT_CONTROLLER_DEVICE to a persistent /dev/serial/by-id entry}"
  fi
  if [[ "$launch" == "read_only_telemetry.launch.py" || "$launch" == "hardware_integration.launch.py" || "$launch" == "sensor_diagnostics.launch.py" ]]; then
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
    "$image" bash -lc "source /opt/ros/jazzy/setup.bash && source /opt/robotics/install/setup.bash && exec ros2 launch robotics_bringup $launch config:=/var/lib/robotics/config/hardware.yaml" "$@"
}
run_capture() {
  : "${LEFT_CAMERA_DEVICE:?set LEFT_CAMERA_DEVICE to a persistent video path}"
  : "${RIGHT_CAMERA_DEVICE:?set RIGHT_CAMERA_DEVICE to a persistent video path}"
  [[ "$LEFT_CAMERA_DEVICE" == /dev/v4l/by-id/* || "$LEFT_CAMERA_DEVICE" == /dev/v4l/by-path/* ]]
  [[ "$RIGHT_CAMERA_DEVICE" == /dev/v4l/by-id/* || "$RIGHT_CAMERA_DEVICE" == /dev/v4l/by-path/* ]]
  [[ -e "$LEFT_CAMERA_DEVICE" && -e "$RIGHT_CAMERA_DEVICE" ]]
  [[ "$(readlink -f "$LEFT_CAMERA_DEVICE")" != "$(readlink -f "$RIGHT_CAMERA_DEVICE")" ]]
  local width="${1:?width required}" height="${2:?height required}" fps="${3:?fps required}"
  local tolerance="${4:?maximum pair delta in milliseconds required}" count="${5:?pair count required}"
  local name="${6:-capture_$(date -u +%Y%m%dT%H%M%SZ)}" format="${7:-}"
  [[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]]
  prepare_data
  docker run --rm -it --network host \
    --device "$LEFT_CAMERA_DEVICE:$LEFT_CAMERA_DEVICE" \
    --device "$RIGHT_CAMERA_DEVICE:$RIGHT_CAMERA_DEVICE" \
    --mount "type=bind,src=$data/calibration,dst=/var/lib/robotics/calibration" \
    "$image" bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/robotics/install/setup.bash && exec ros2 run stereo_perception capture_stereo "$@"' \
    capture_stereo "$LEFT_CAMERA_DEVICE" "$RIGHT_CAMERA_DEVICE" "$width" "$height" "$fps" "$tolerance" "$count" "/var/lib/robotics/calibration/$name" ${format:+"$format"}
}
run_calibration() {
  local name="${1:?capture name required}" columns="${2:?inner columns required}"
  local rows="${3:?inner rows required}" square="${4:?square size in meters required}"
  [[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]]
  : "${LEFT_CAMERA_DEVICE:?set LEFT_CAMERA_DEVICE to the captured left identity}"
  : "${RIGHT_CAMERA_DEVICE:?set RIGHT_CAMERA_DEVICE to the captured right identity}"
  prepare_data
  [[ -f "$data/calibration/$name/manifest.csv" ]]
  awk -F, -v left="$LEFT_CAMERA_DEVICE" -v right="$RIGHT_CAMERA_DEVICE" \
    'NR > 1 && ($2 != left || $3 != right) { exit 1 }' "$data/calibration/$name/manifest.csv" || {
      echo "camera identities disagree with capture manifest" >&2; return 2;
    }
  docker run --rm --network host \
    --mount "type=bind,src=$data/calibration,dst=/var/lib/robotics/calibration" \
    "$image" bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/robotics/install/setup.bash && exec ros2 run stereo_perception calibrate_stereo "$@"' \
    calibrate_stereo "/var/lib/robotics/calibration/$name/left_*.png" "/var/lib/robotics/calibration/$name/right_*.png" \
    "$columns" "$rows" "$square" "/var/lib/robotics/calibration/$name/stereo.yaml" \
    "$LEFT_CAMERA_DEVICE" "$RIGHT_CAMERA_DEVICE"
}
case "${1:-}" in
  build)
    build_image
    ;;
  start)
    prepare_data
    if docker container inspect "$container" >/dev/null 2>&1; then
      docker start "$container"
      exit 0
    fi
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
  capture)
    shift
    run_capture "$@"
    ;;
  calibrate)
    shift
    run_calibration "$@"
    ;;
  telemetry)
    run_hardware read_only_telemetry.launch.py
    ;;
  integration)
    run_hardware hardware_integration.launch.py
    ;;
  sensors)
    run_hardware sensor_diagnostics.launch.py
    ;;
  diagnose)
    "$(dirname "$0")/pi_diagnostics.sh"
    ;;
  update)
    build_image
    if docker container inspect "$container" >/dev/null 2>&1; then
      docker stop "$container" || true
      docker rm "$container"
    fi
    "$0" start
    ;;
  *)
    echo "usage: $0 {build|start|stop|logs|shell|capture WIDTH HEIGHT FPS DELTA_MS COUNT [NAME] [FOURCC]|calibrate NAME COLUMNS ROWS SQUARE_M|camera|telemetry|sensors|integration|diagnose|update}" >&2
    exit 2
    ;;
esac
