# Raspberry Pi 5 deployment and bench preparation

The release image uses the ROS 2 Jazzy Ubuntu 24.04 base for `amd64` and `arm64`.
Build on the Raspberry Pi 5 for the ARM64 deployment milestone. A build on
another architecture, including emulated ARM64, does not validate Pi USB,
timing, power, or flight-controller behavior. The `.devcontainer` remains the
development environment; `Dockerfile` and `scripts/pi_deploy.sh` are the
deployment path. Neither starts autonomous flight.

## Prepare the Pi

1. Install 64-bit Ubuntu 24.04 on the Pi 5. Verify `uname -m` reports
   `aarch64` and `/etc/os-release` reports Ubuntu 24.04. Set a stable clock,
   cooling, and a power supply sized for the Pi and USB devices.
2. Install Docker Engine using Docker's current Ubuntu instructions. Confirm
   `docker info` works and reports `aarch64` on the Pi. Install `v4l-utils`
   and `usbutils` on the host for device inspection. For a native install,
   grant the operating user the host `video` and `dialout` groups and log in
   again. The container commands map only explicitly selected devices and
   run as root inside the container.
3. Clone this repository. Run `scripts/pi_deploy.sh build` on the Pi. The
   release image installs architecture-neutral Ubuntu/ROS dependencies and
   compiles the C++17 packages natively. The script records the image ID,
   architecture, and source revision in `deployment_data/build-info.txt`.
   Use `docker run --rm
   robotics-jazzy:local bash -lc 'source /opt/ros/jazzy/setup.bash; source
   /opt/robotics/install/setup.bash; colcon list --base-paths
   /opt/robotics/workspace/src'` to inspect package discovery.

The build script checks that the host and Docker daemon architectures match,
selects the native platform explicitly, and limits compilation to two jobs
with sequential package builds. CI also builds the complete release image on
an ARM64 runner; its execution must be checked before treating ARM64 compilation
as verified. The AMD64 development path is retained.

## Development and updates

Use `.devcontainer` for AMD64 development; `compose.yaml` runs the inert
release diagnostics image. The release
Dockerfile uses distribution Python packages, including OpenCV, NumPy, SciPy,
PyYAML, and Matplotlib, avoiding wheel availability assumptions on ARM64.
Run `scripts/pi_deploy.sh update` only after reviewing changes. It rebuilds
and replaces the diagnostics container; calibration, config, and logs remain
in `deployment_data/`. The script does not enable hardware output.

## Identify and calibrate cameras

1. Run `v4l2-ctl --list-devices`, `ls -l /dev/v4l/by-id`, and
   `ls -l /dev/v4l/by-path`. Record a distinct persistent path for each
   OV9281. If the devices have no unique serial numbers, use stable USB port
   paths and preserve port assignment. Inspect supported formats with
   `v4l2-ctl -d DEVICE --list-formats-ext`. Verify resolution, frame rate,
   exposure, and USB bandwidth on the actual Pi.
2. Start diagnostics with `scripts/pi_deploy.sh start`. The first run copies
   `src/robotics_bringup/config/hardware.yaml` to
   `deployment_data/config/hardware.yaml`. Edit that persistent copy with the
   selected paths and measured settings. Set a calibrated software pairing
   tolerance. Host monotonic receipt timestamps are paired; USB cameras have
   no established hardware synchronization here.
3. Set `LEFT_CAMERA_DEVICE` and `RIGHT_CAMERA_DEVICE` to the selected persistent
   paths. Capture stationary checkerboard pairs at varied positions and tilts:
   `scripts/pi_deploy.sh capture 640 480 20 5 20 board_run`
   (add a four-character format argument only after confirming it with
   `v4l2-ctl --list-formats-ext`).
   This installed C++ executable prompts before each pair and saves a CSV
   manifest with both device paths and host monotonic timestamps. It rejects
   pairs beyond the tolerance. It maps the two selected devices into the
   container and writes `deployment_data/calibration/board_run/`. Use at least eight usable
   pairs with measured checkerboard square size.
4. Run `scripts/pi_deploy.sh calibrate board_run 7 5 0.04`, replacing the board
   dimensions and measured square size. This estimates
   individual intrinsics, distortion, stereo rotation/translation, and
   rectification matrices, and rejects poor reprojection quality. The identity
   strings come from `LEFT_CAMERA_DEVICE` and `RIGHT_CAMERA_DEVICE` and must
   exactly match the device paths used for acquisition. Inspect
   `deployment_data/calibration/board_run/stereo.yaml` before
   pointing both camera and perception parameters at it. No baseline is
   assumed before calibration.
5. Set both calibration paths in `deployment_data/config/hardware.yaml` to
   `/var/lib/robotics/calibration/board_run/stereo.yaml`. Measure the transform
   from the body frame to the left optical frame. Fill the translation in body
   coordinates in meters and the Rodrigues rotation offset from the REP 103
   optical convention in radians.
   The external perception node refuses missing mounting parameters. The
   configured initial position is a local VO origin, not a surveyed global
   pose. With paths, calibration, and mounting set, run
   `LEFT_CAMERA_DEVICE=... RIGHT_CAMERA_DEVICE=... scripts/pi_deploy.sh camera`.

The UVC node acquires each camera on its own thread, maps host monotonic
receipt time into the ROS clock domain, publishes raw mono images and raw
`CameraInfo` on matching topics, and reports
read failures, estimated receipt gaps, and reconnects. The perception node
rejects stale images and pairs with timestamp difference above
`max_pair_delta_s`. It publishes calibrated camera info and rectified data.
OpenCV does not expose a reliable camera device sequence or exposure timestamp
here; the reported sequence is the acquisition count, and gap counts are
estimates. Hardware synchronization remains unverified.

## Read-only flight-controller discovery

Inspect the actual controller board, firmware, and USB serial identity before
editing `serial_device` and setting `protocol: legacy_msp_v1_candidate` and
`transport_enabled: true`. The archived controller code shows MSP v1 `IDENT`,
`STATUS`, `ANALOG`, and `RAW_IMU` queries, but the live controller version is
unknown. Run `FLIGHT_CONTROLLER_DEVICE=... scripts/pi_deploy.sh telemetry` with
propellers removed. This node sends only those read-only requests, checks MSP
framing and checksums, and publishes raw fields and diagnostics. Its ROS stamp
is host poll completion, not a controller sensor timestamp. Raw IMU counts
are deliberately not converted to SI units or fused until the actual sensor
scale, axes, covariance, timing, and gravity convention are measured. It never
sends `MSP_SET_RAW_RC` or changes a mode. MSP v1 has no transaction ID; the
transport matches each response to its requested command and uses bounded
timeouts.
Record the board, firmware family, and firmware version in the configuration
before integrated bring-up. These are operator inventory fields; the telemetry
probe still treats physical control behavior as unverified.

## Startup, shutdown, and diagnostics

`scripts/pi_deploy.sh start` runs diagnostics only. `scripts/pi_deploy.sh
stop` stops it. `scripts/pi_deploy.sh logs` displays its logs. Run
`scripts/pi_deploy.sh diagnose` to save OS, USB, device, supported V4L2 modes,
serial permissions, Docker, and ROS graph
information under `deployment_data/diagnostics/`. In a running container,
inspect `ros2 node list`, `ros2 topic list -t`, `ros2 topic echo
/diagnostics`, `ros2 topic hz /camera/left/image_raw`, and `ros2 topic hz
/camera/right/image_raw`. `hardware_diagnostics.launch.py` starts the
read-only telemetry and gate diagnostics nodes without cameras. With all
camera and read-only transport settings supplied, `scripts/pi_deploy.sh sensors`
starts stereo localization, raw telemetry, and the closed gate without a
planner/controller. Inspect `/perception/diagnostics`, `/visual_odometry`,
`/perception/state_estimate`, and `/flight_controller/raw_telemetry`; raw IMU
counts remain unfused. With all
measured settings supplied, `scripts/pi_deploy.sh integration` starts the
cameras, stereo estimator, read-only telemetry, and existing autonomy node
without its simulator. No mission goal is submitted. The physical command
gate observes commands and reports a closed state; it cannot transmit
actuator commands. Stop foreground hardware launches with Ctrl-C.

If a camera is missing, check persistent path resolution, container mapping,
USB power, and `v4l2-ctl` negotiated modes. If calibration is refused, inspect
camera identities, image dimensions, checkerboard detection, and stored
matrices. If MSP `IDENT` is absent, check the exact board firmware, UART/USB
port, baud, wiring, and host permissions before changing protocol settings.
Do not infer compatibility from the archived Python bridge.
After USB re-enumeration changes a device minor number, stop and restart the
foreground container launch to recreate its explicit device mappings. Native
acquisition retries the persistent path; Docker mappings may retain the old
device number. Verify both recovery paths on the bench.

## Physical verification remaining

Native Pi image execution, two-camera modes and timing, physical calibration,
mounting transform, firmware identification, raw sensor scale/covariance,
serial reconnect behavior, receiver and failsafe configuration, channel
mapping, arming authority, and safe physical response to lost localization
all require bench measurements. Physical command encoding and actuator
transmission remain intentionally absent. Complete propeller-free and
supervised staged validation separately before any flight authorization.

The retained Phase 4 dynamic failure trace in `results/perception/dynamic/`
shows valid VO with roughly 0.01 m position error through the last recorded
frame; the autonomy tracker then rejected a 0.519 s stale state during
replanning. It does not establish the earlier reported VO jump's cause.
The estimator now rejects malformed PnP transforms and nonfinite or unsupported
inlier/error results before pose composition. Without a calibrated VO noise
model, odometry covariance is explicitly unknown; a chi-square innovation
gate would present unsupported confidence. Localization loss still stops
navigation and requires explicit estimator reset.
