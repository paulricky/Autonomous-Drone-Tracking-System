# ROS 2 migration report

## Audit and baseline

The original active system was a 2,536-line installable Python library with 57
pytest tests, deterministic point-mass/MuJoCo simulation, geometric and
kinodynamic A*, an EGO-inspired optimizer, B-splines, tracking, stereo
processing, voxel mapping, and an inert MSP encoder. The recovered
`pidrone_pkg` was a separate archived ROS 1 application. Baseline on macOS:
`57 passed in 12.70s`; Git was clean on `main`.

Problems were the absence of ROS 2 package metadata, native libraries, launch
and interface packages, reproducible Jazzy tooling, CI, and a safe ROS-facing
hardware boundary. The legacy browser dashboard also dominated language
statistics despite not being used.

## JavaScript cleanup

All tracked JavaScript was under the recovered dashboard: 18 files totaling
2,304,546 bytes. Largest files were `three.js` (813,502),
`bootstrap.bundle.js` (212,345), `Chart.bundle.min.js` (207,839),
`ColladaLoader2.js` (156,085), and `ros3d.js` (153,596). All 18 were removed;
zero JavaScript files remain. The original first-party `main.js` depended on
the obsolete ROS 1 ROSBridge/MJPEG stack. RViz2, diagnostics, ROS 2 CLI, and
rosbag2 are its intended replacements. HTML/CSS and an archive notice remain
for provenance; the preserved microSD image is the authoritative full copy.

## Implemented migration

- One colcon workspace rooted at the repository.
- Jazzy interface, C++ core, Python algorithm, mission, and bringup packages.
- C++17 MSP encode/decode and bounded legacy RC channel encoding, derived from
  the recovered implementation without opening serial hardware.
- C++ feedback-plus-feedforward tracker with state freshness, finite-value,
  tracking-error, and vector saturation gates.
- C++ bounded stereo timestamp synchronizer with deterministic stale-frame
  rejection, monotonicity checks, queue limits, and diagnostics counters.
- Hardware-disabled lifecycle diagnostics and deterministic mission states.
- Standard frames, SI units, installed YAML, and Python launch infrastructure.
- Ubuntu 24.04/Jazzy dev container, production container, Compose, clang tools,
  AMD64 CI, and advisory hosted ARM64 validation.

## Preserved and pending

The native C++17 ROS 2 path is the active planning, trajectory, tracking,
stereo, mapping, and estimation implementation. The redundant Python
kinodynamic planner, optimizer, B-spline, tracker, dynamics, stereo-depth,
estimator, and MSP modules were removed. Offline Python geometric planning,
voxel mapping, and waypoint/MuJoCo runners remain outside that path as offline
simulation and visualization tools. ROS 1 flight/fault/state-estimation source remains
archive-only because its controller conventions and failure behavior require
hardware review. Physical camera calibration, Pi 5 runtime performance, and
verified MSP flight-controller behavior remain unmeasured.

## Executed validation (2026-10-26, Intel macOS)

Commands actually executed:

```bash
.venv/bin/python -m pytest -q
# 57 passed in 12.70s (pre-migration baseline)

g++ -std=c++17 -Wall -Wextra -Wpedantic -Isrc/robotics_core/include \
  -c src/robotics_core/src/msp.cpp -o /tmp/robotics_tools-cpp-check/msp.o
g++ -std=c++17 -Wall -Wextra -Wpedantic -Isrc/robotics_core/include \
  -c src/robotics_core/src/trajectory_tracker.cpp \
  -o /tmp/robotics_tools-cpp-check/trajectory_tracker.o
# both compiled without diagnostics

PYTHONPATH=src/flight_mission .venv/bin/python -m pytest -q \
  src/flight_mission/test tests
# 60 passed in 12.06s

MPLBACKEND=Agg .venv/bin/python scripts/run_duckiedrone_sim.py --scenario baseline
# complete; goal reached; 0 collisions; final error 0.09244 m
```

Package XML parsing, Python byte compilation, and `git diff --check` also
passed. Docker validation subsequently completed on the same host. Discovery
is explicitly scoped to `src`, and `archive/COLCON_IGNORE` prevents the ROS 1
Catkin snapshot and its embedded Python environment from entering the graph.
The root setuptools compatibility shim was removed; `pyproject.toml` remains
the canonical non-ROS editable-install metadata.

```bash
docker compose --progress plain build
# Summary: 6 packages finished

docker compose run --rm robotics_stack bash -lc \
  'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   colcon list --base-paths src && colcon test --base-paths src && \
   colcon test-result --verbose'
# robotics_tools, robotics_bringup, robotics_core,
# flight_interfaces, flight_mission, stereo_perception
# Summary: 17 tests, 0 errors, 0 failures, 0 skipped
```

The live diagnostic launch reached lifecycle state `active`, published
`sp-racing-f3-unconnected`, `command_gate: closed`, and mission state `idle`.
No serial device was opened. RViz2 and rosbag2 visualization/replay remain
unexercised because the diagnostic-only launch publishes neither visual data
nor bag fixtures.

The existing measured navigation results remain in `BENCHMARKS.md`: global
planning 3.574–7.558 s, local optimization 0.255–5.028 s, tracking RMS
0.0669–0.0929 m, minimum measured clearance 0.237 m, and dynamic replanning
13.824 s. These are macOS deterministic-simulation results, not native C++,
ROS transport, Pi 5, camera, or aircraft results. Perception throughput,
mapping update rate, peak memory, CPU percentage, and end-to-end latency remain
unmeasured; no values are invented.
# Phase 3 native planning update (2026-10-26)

The simulation path now has native C++17 Kinodynamic A*, B-spline,
EGO-inspired optimization, dynamics, and tracking components. The optimizer
follows EGO-Planner's ESDF-free
control-point cost decomposition, but is an adaptation using occupied-voxel
nearest-point penalties and backtracking gradient descent, not the upstream
rebound optimizer or topology search.

Core classes contain no ROS types. New boundary messages are
`TrajectoryPoint`, `TimedTrajectory`, `VehicleState`, and `ControlCommand`.
Physical MSP output, arming, motors, and serial access remain excluded.

# Phase 4 implementation checkpoint — 2026-10-27

Native stereo processing, geometry-based rendering, PnP RANSAC odometry and a
timestamp/quality-checking state estimator are now connected to the existing
voxel mapper and navigation pipeline. The mission manager submits the existing
navigation action after map/state readiness. Redundant Python implementations
have since been removed. C++ sources/headers in active
packages are formatted with the repository style; archives were not touched.

The renamed Jazzy workspace has six packages. The rebuilt Dev Container
completed rosdep resolution; its isolated Python environment imported ROS 2
modules, NumPy 1.26.4, and SciPy 1.16.2. Focused native tests passed (27 tests),
and focused Python tooling tests passed (13 tests). Software-only static and
dynamic stereo navigation succeeded with one and two trajectories respectively;
the disconnect scenario stopped with braking and an explicit localization/map
failure. These results do not establish physical flight readiness or complete
physical camera or flight-controller integration.

The earlier compiler error was a `cv::MatCommaInitializer_` passed directly to
`warpAffine`; materializing a `cv::Mat` fixed it. The VehicleState discovery
error came from sourcing the overlay in a backgrounded shell chain, not failed
IDL generation. Source the ROS installation and workspace overlay in the shell
that executes the ROS command.

Targeted native tests and static closed-loop ROS execution have passed. Full
parity, long-run stability, dataset/hardware validation and performance campaigns
are Phase 5 work. See [current stereo implementation](STEREO_PERCEPTION.md).
The sections above retain the earlier migration history.
