# Development handoff

## Current state

2026-10-28: Raspberry Pi deployment and non-actuating hardware software preparation
completed in this working tree, based on commit `c16c231`. Changes are uncommitted.
Physical readiness and native ARM64 compilation remain unverified.

## Implemented

- Native-platform Docker builds check host/daemon architecture, use sequential
  packages and two compiler jobs, and preserve AMD64 development. ARM64 CI now
  builds the six ROS packages rather than only testing Python tooling.
- Persistent Pi configuration/calibration/log storage, capture and calibration
  commands, device inventory with V4L2 modes/USB identity/serial permissions,
  restartable diagnostics, and explicit camera/serial mappings.
- Concurrent calibration pair reads with configurable timestamp tolerance,
  optional FourCC, distinct underlying device checks, and protection against
  overwriting capture data. Calibration requires distinct camera identities,
  finite target dimensions, and acceptable individual/stereo reprojection RMS.
- Raw images have corresponding measured CameraInfo. Physical acquisition and
  perception reject synthetic calibration. Preflight checks device paths,
  shared calibration, mounting, host clock, read-only transport, firmware
  inventory, and disabled autonomous/output settings before device startup.
- Duplicate camera timestamps are rejected. Host receipt timestamps remain
  explicitly different from exposure timestamps; independent USB cameras are
  software matched. Device sequence numbers/drop counts remain unavailable
  through OpenCV; acquisition counts and estimated receipt gaps are reported.
- MSP discards timed-out partial/queued replies before a new query and explains
  explicit IDENT rejection. The existing parser/allowlist/virtual serial path
  remain read-only. Aggregate timestamps now match host poll completion.
  Raw controller IMU counts are not fused.
- `sensor_diagnostics.launch.py` / `pi_deploy.sh sensors` combine stereo
  localization, read-only telemetry, and the closed gate without navigation.
- Physical gate diagnostics reject replayed command observations, nonfinite
  state data, and malformed trajectory samples. Protocol/configuration/mode
  verification remains false; no physical actuator transport exists.
- Replanning yields to the executor before tracking with a new state sample.
  This addresses use of the pre-plan snapshot after blocking planning; braking
  and existing freshness thresholds are preserved. The retained historical
  trace does not prove a PnP/VO defect or explain every scheduling delay.
- Functional naming replaces active milestone identifiers. Standard `build/`,
  `install/`, `log/` directories are used and already ignored by Git. Newly
  created milestone-suffixed artifacts and obsolete Python cache were removed.

## Modified files

- Deployment: `Dockerfile`, `.github/workflows/ros2-ci.yml`,
  `scripts/pi_deploy.sh`, `scripts/pi_diagnostics.sh`.
- Bring-up: `src/robotics_bringup/{package.xml,config/hardware.yaml}`,
  `robotics_bringup/hardware_config.py`, launch files `uvc_stereo`,
  `read_only_telemetry`, `hardware_integration`, `sensor_diagnostics`, and tests
  `test_hardware_config.py`, `test_launch_description.py`.
- Core: `include/robotics_core/msp.hpp`, source files `autonomy_sim_node.cpp`,
  `msp_serial.cpp`, `msp_telemetry_node.cpp`, `physical_command_gate_node.cpp`.
- Stereo: `CMakeLists.txt`, source files `calibrate_stereo.cpp`,
  `capture_stereo.cpp`, `perception_node.cpp`, `stereo_pipeline.cpp`,
  `stereo_synchronizer.cpp`, `uvc_stereo_node.cpp`, synchronizer test.
- Naming/references: `scripts/validate_ros_integration.py` (renamed),
  `scripts/{setup_environment,run_planning_sim}.py`, `README.md`,
  `docs/{PI_DEPLOYMENT,navigation_development_history}.md` (renamed),
  `docs/{HARDWARE_READINESS,RASPBERRY_PI_5,ROS2_ARCHITECTURE,STEREO_PERCEPTION}.md`.

## Actual validation

- The interrupted full AMD64 Jazzy workspace build completed successfully:
  six packages in 3 min 15 s, using standard Colcon directories.
- Focused Colcon tests subsequently reported 43 tests, zero failures/errors,
  including MSP virtual serial, command gates, perception, and launch preflight.
  Colcon's aggregate includes CTest wrapper results.
- Final incremental core/stereo builds passed after correcting telemetry
  timestamp placement, CameraInfo rectification metadata, and mandatory saved
  calibration quality metadata. The calibration-only rerun reported 42
  aggregate tests with zero failures/errors (one fewer CTest wrapper selected).
  Runtime imports of OpenCV 4.6.0, NumPy, SciPy, PyYAML and Matplotlib succeeded
  on AMD64; all camera and core executables were listed from the overlay.
- The installed sensor diagnostics launch and preflight/launch-description
  tests passed after their final update; aggregate remained 42 with zero
  failures/errors. Final shell syntax, naming/reference and whitespace checks
  passed. Git ignore rules and their existing exceptions were preserved.
- Targeted dynamic simulation succeeded with two trajectories, 0.0978 m final
  truth goal error and zero sampled collisions. Unconfigured camera launch
  failed before any images or trajectories. Results: `log/ros_integration/`.
- Shell syntax, diff whitespace and active naming/reference searches passed.
- ROS Noble base manifest contains `linux/arm64/v8`; 13 required ROS Jazzy
  packages were found in the official Noble ARM64 apt index. HTTPS metadata
  retrieval had a hostname certificate error; the repository HTTP endpoint
  succeeded. Package installation still uses apt signature verification.

## ARM64 and hardware blockers / next steps

1. On Ubuntu 24.04 ARM64 Pi, run `scripts/pi_deploy.sh build`; retain build-info
   and logs. Check native ARM64 CI. No ARM64 Colcon completion is claimed.
   The earlier emulated interruption occurred during package setup before
   Colcon; retained evidence cannot distinguish emulator overhead, resource
   pressure or a package-setup stall. No architecture-specific source/CMake
   defect was established. Repeated emulated builds were avoided.
2. Inventory both physical cameras; measure supported modes, USB bandwidth,
   exposure timing and receipt skew. Capture/calibrate, inspect RMS and
   identities, measure mounting, then run camera bring-up. Validate USB
   disconnect/reconnect and container remapping (see `PI_DEPLOYMENT.md`).
3. Inventory actual controller/firmware, select the persistent serial device
   and explicit candidate MSP mode, then run read-only telemetry. Verify
   firmware compatibility, timeouts and reconnect behavior on the bench.
4. Measure IMU units, axes, timing, extrinsics and covariance before adding an
   SI measurement conversion/fusion path. VO covariance remains unmeasured;
   no unsupported confidence model is fabricated.
5. Run non-actuating integrated bring-up with measured config. Controller
   control mapping, failsafe response, arming authority and physical flight
   remain unverified and gated. No physical device was accessed in this pass.
