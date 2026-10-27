# Native stereo perception — Phase 4 implementation

The deployed path is C++17. Python perception/mapping/localization modules remain
reference code; the ROS perception navigation launch does not import them.

`ece3162_perception` provides ROS-independent acquisition, synchronization,
calibration, rectification, SGBM, Q-matrix depth reconstruction, back-projection,
stereo visual odometry, state estimation, and fixed-world synthetic rendering.
The existing `ece3162_core::VoxelMap` performs batch free-ray/occupied-endpoint
integration, unknown-space queries, and inflation. Endpoints win over free rays
within an observation; only observed rays clear previous occupancy. Unseen cells
are not automatically cleared. The planner and optimizer read this same map.

## Input modes and launch

After sourcing `/opt/ros/jazzy/setup.bash` **and** `install/setup.bash`:

```bash
ros2 launch ece3162_bringup synthetic_perception.launch.py
ros2 launch ece3162_bringup stereo_localization.launch.py
ros2 launch ece3162_bringup perception_navigation.launch.py
ros2 launch ece3162_bringup perception_navigation.launch.py dynamic_after_s:=3.0
ros2 launch ece3162_bringup perception_navigation.launch.py disconnect_after_frames:=30
ros2 launch ece3162_bringup perception_navigation.launch.py low_texture_after_frames:=30
ros2 launch ece3162_bringup uvc_stereo.launch.py left_device:=/dev/v4l/by-id/LEFT right_device:=/dev/v4l/by-id/RIGHT calibration_file:=/path/stereo.yaml
```

Standalone perception/localization nodes run until stopped. Mission launches
terminate at success or explicit failure. Missing UVC configuration is a startup
error, not an empty/clear environment. Camera launch never starts a flight controller.
Use Linux device permissions and narrowly scoped `--device` mappings for only
the two video devices; privileged containers and serial-device mappings are unnecessary.

`uvc_stereo_node` publishes `camera/{left,right}/image_raw`; `perception_node`
in `external` mode uses the same native processing path as synthetic and recorded
input. It retains actual image payloads by sequence through bounded timestamp
pairing. Acquisition uses OpenCV V4L2, persistent configured device paths, selected
resolution/rate/pixel format and optional exposure/gain. Unsupported controls are
reported. Timestamps are host receipt times mapped from monotonic to ROS time;
they are **not verified exposure times**. Drop counts are receipt-gap estimates,
not verified driver sequence counters. Physical disconnect/blocking behavior still
needs camera tests. Global shutter does not establish stereo synchronization.

Recorded mode is `ros2 run ece3162_perception perception_node --ros-args -p mode:=recorded
-p calibration_file:=/path/stereo.yaml -p recorded_manifest:=/path/pairs.txt`.
Each manifest row is `capture_seconds left_image_path right_image_path`, ordered
strictly by timestamp. Paths may be relative to the manifest. Replay preserves
capture intervals and translates the sequence epoch into ROS time.

## Calibration and geometry

`ros2 run ece3162_perception calibrate_stereo 'left/*.png' 'right/*.png' 7 5 0.04 stereo.yaml`
uses sorted paired images, checkerboard internal-corner counts and measured square
size in metres. The CLI requires eight usable pairs and RMS <= 1 pixel. Capture
diverse positions, distances and tilts; a low reprojection error alone does not
prove physical coverage or accuracy. YAML stores K/D/R/T, rectification rotations,
projection matrices, Q, resolution and provenance. Runtime rejects malformed
calibration and image-resolution mismatch. Synthetic calibration is explicitly
synthetic, never presented as measured OV9281 calibration.

Rectification maps are cached. SGBM rejects invalid disparities and uses uniqueness,
speckle and disparity consistency settings. Depth uses the calibrated Q matrix;
the standard parallel case reduces to Z=fB/d. Uncertainty grows approximately as
sigma_Z=abs(fB/d²)*sigma_d. Point clouds use rectified projection parameters.

`map`/`odom` use x-forward, y-left, z-up simulation axes; optical axes are
x-right, y-down, z-forward. The map-to-odom transform is identity, **not a drift
correction**. The tree includes base_link, left/right_camera_link, optical and
rectified frames. Camera mounting is configurable with
`body_from_left_optical_translation` (metres) and
`body_from_left_optical_rotation_vector` (radians relative to optical-to-FLU).
`initial_body_position` fixes the initial coordinate origin. Defaults describe
the synthetic rig, not measured physical mounting. Rectification and calibrated
right-camera transforms are included. Mapping obtains the sensor origin from TF
at the cloud timestamp, not from an assumed body-center origin.

## Localization and navigation

Native VO detects Shi–Tomasi features, tracks forward/backward with LK, reconstructs
previous-frame metric points from stereo depth, estimates motion using PnP RANSAC,
refines inliers with LM, and propagates pose. The native estimator checks timestamps,
motion jumps and tracking validity, smooths differentiated velocity, and publishes
orientation through Odometry. Covariance is an explicitly heuristic accumulated
uncertainty, not a calibrated posterior. This is stereo odometry, without IMU,
loop closure, global relocalization or SLAM. Tracking loss latches invalid until
explicit reinitialization; there is no ground-truth fallback.

The mission manager sends `navigate_to_pose` only after fresh map and state
messages. `autonomy_pipeline` plans and optimizes against perception occupancy;
its tracker consumes only `perception/state_estimate`. `simulated_drone` integrates
acceleration commands and feeds truth solely to rendering/evaluation. The synthetic
mission explicitly selects `unknown_is_free=true` in a bounded test volume.
The default perception-map policy is conservative unknown-as-occupied; exploratory
navigation through unknown space has not been validated.

Map changes revalidate the remaining spline immediately. A rejected replacement
or invalid/stale localization produces explicit failure and simulated bounded
braking. No collision-ignore cooldown is used. The hardware lifecycle monitor
continues reporting a closed command gate.

Sensor image/cloud/state topics use best-effort bounded QoS. Trajectories are
reliable/transient-local with absolute ROS start stamps and relative sample times.
`visual_odometry`, `perception/diagnostics`, `mapping/occupied_voxels`,
`planning/status`, `mission/status` and `mission/manager_status` expose runtime
state. `mission/cancel` (std_msgs/Empty) requests action cancellation.

## Validation boundary

Targeted native tests cover depth, matrix validation, stereo motion/rotation,
state freshness/loss and batch mapping semantics. A static ROS mission using
perception occupancy and PnP estimates has succeeded; sensor-loss smoke tests
have produced explicit failures. Existing `scripts/validate_phase4_ros.py` and
native `perception_benchmark` retain hooks for Phase 5. Comprehensive parity,
calibration coverage, recorded datasets, drift, mapping accuracy, overload,
long-duration stability, Pi 5 performance and physical OV9281 operation remain
validation work. No real camera or Raspberry Pi measurements are claimed.
