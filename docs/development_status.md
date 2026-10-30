# Development handoff

## Current physical bring-up and landing status (2026-10-29)

**NOT READY for flight.** The current clean baseline `cc6c210` contains
Pi-local `/mission/land`, `/mission/disarm`, `/mission/prepare_shutdown`,
navigation cancellation/inhibition, a controller-independent landing state
machine, read-only MSP API/variant/version/board inventory, and an explicit
`/preflight/check` report. The production landing adapter is deliberately
unavailable because the connected flight-controller firmware and its native
landing/landed indication have not been identified. It sends no mode, RC,
arm, disarm, motor or landing command. Physical output is still disabled.

Landing status is requested, acknowledged landing, confirmed landed,
confirmed disarmed, or failed. Navigation remains inhibited after LAND,
including after rejection or a repeated request. Disarm requires fresh
authoritative ground/landed feedback; shutdown preparation additionally
requires fresh confirmed disarming and explicit operator authorization, false
by default. `/mission/prepare_shutdown` is only an interlock query and never
halts the Pi. ROS parameter access does not independently authenticate an
operator, so manual/pilot recovery and access control remain unverified.

The interrupted Docker failure was a real readiness-node defect, not a Docker,
package-discovery or MSP dependency issue: assigning an integer to Jazzy's
byte-valued `diagnostic_msgs/DiagnosticStatus.level` caused a C conversion
assertion. The node now uses the message's byte constants. The later AMD64
Docker check passed 31 `flight_mission` tests, including mocked landing,
live navigation inhibition/preflight, and three virtual-serial MSP inventory
cases. Targeted core MSP/serial/command-gate tests, five bring-up tests and
nine mocked Pi-deployment tests also passed. The image is AMD64 on a macOS
x86_64 host; this is not native ARM64/Pi validation.

The continuation added an explicit shutdown authorization interlock, false by
default, and separate PASS/FAIL/UNVERIFIED entries for camera operation,
controller identity, MSP compatibility and operator identity. The focused
AMD64 rebuild of `flight_mission` and `robotics_bringup` passed. All 31 mission,
five bring-up and nine mocked deployment tests passed; targeted core MSP,
virtual serial and command-gate tests passed in the earlier run. After the
operator-identity report refinement, the focused `flight_mission` rebuild and
24 landing/readiness tests passed. Native Pi
compilation, camera
acquisition/calibration/mounting, firmware/protocol/control-mode and landing
compatibility, IMU metadata, localization covariance/health, independent
pilot recovery, failsafes, motor configuration and power all require measured
hardware evidence. Use [the ordered propeller-free bring-up procedure]
(PROPELLER_FREE_BRINGUP.md): `scripts/pi_deploy.sh build`, `start`, `preflight`,
`diagnose`, then camera capture/calibration and read-only telemetry on the Pi.
No powered flight or motor test is authorized by these software results.

## Current state

2026-10-28: Prior deployment and naming work is committed as `2e1f430`.
The follow-up reliability fixes below are uncommitted. No further software-only
blocker to attempting native Pi deployment was identified. Physical readiness
and native ARM64 compilation remain unverified.

## Latest reliability fixes and validation

- Fixed a confirmed mid-poll serial-disconnect failure: subsequent read-only
  queries now return unavailable instead of throwing and terminating telemetry.
  Diagnostics explicitly report disconnect/reconnect pending. Invalid query
  commands and timeouts still throw; physical output remains unavailable.
- Deployment now reports missing/invalid devices and capture names clearly,
  loads ROS and workspace overlays in its interactive shell, honors custom
  persistent storage for diagnostics, and resolves helper paths when invoked
  from outside the repository. Existing operator config is preserved.
- Diagnostics bound each external command to 15 seconds, record failures and
  continue collecting evidence after V4L2/USB/udev errors. Serial permission
  inspection follows the persistent symlink to the actual device.
- Changed files: `scripts/pi_deploy.sh`, `scripts/pi_diagnostics.sh`,
  `src/robotics_core/include/robotics_core/msp_serial.hpp`,
  `src/robotics_core/src/{msp_serial,msp_telemetry_node}.cpp`,
  `src/robotics_core/test/test_msp_serial.cpp`, and this handoff.
- Incremental AMD64 Jazzy `robotics_core` build passed (20.3 s). Seven selected
  test cases passed across `test_msp`, `test_msp_serial`, `test_command_gate`,
  including persistent-path replacement/reconnect on virtual serial ports.
  Colcon reported 41 aggregate retained results with zero failures/errors;
  unrelated suites were not rerun. Standard `build/`, `install/`, `log/` used.
- Isolated Linux script checks passed for missing devices, mismatched native
  architecture rejection, start/restart, config preservation, sourced shell,
  invocation outside the repository, custom data storage, and continued report
  collection after failed V4L2/USB tools. These used mocked utilities and a
  `/dev/null` fixture, not physical devices. Shell syntax and diff checks passed.
- No broad navigation regression or emulated ARM64 build was repeated: these
  changes affect deployment/transport failure handling, not navigation algorithms.
  Prior ARM64 image/dependency checks remain the available evidence.

Next action: on a Raspberry Pi 5 with Ubuntu 24.04 ARM64, adequate power/cooling
and Docker, run `scripts/pi_deploy.sh build`, then `scripts/pi_deploy.sh start`
and `scripts/pi_deploy.sh diagnose`. Retain the build-info and diagnostic files.
For sensor bring-up, provide both OV9281 cameras with stable USB assignments,
a measured checkerboard and rigid mount, and the actual controller plus its
documented serial/firmware identity. Follow `PI_DEPLOYMENT.md` to capture,
calibrate and configure measured transforms before `camera` or `sensors`.
Keep propellers removed during controller bench work. No physical device was
opened, and no actuator command or navigation goal was sent in this follow-up.

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
