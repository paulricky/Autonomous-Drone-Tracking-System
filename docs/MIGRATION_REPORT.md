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

The tested Python planners, optimizer, B-spline, stereo, mapper, and simulator
remain canonical until equivalent native libraries have numerical parity tests.
They are not duplicated ROS applications. ROS 1 flight/fault/state estimation
source remains archive-only because its controller conventions and failure
behavior require hardware review. No camera, F3, Pi 5, ROS graph, colcon build,
real perception benchmark, or physical flight was claimed in this migration.

Priority remaining work is: run the dev container and colcon CI; add numerical
parity fixtures before native planner/map/stereo replacement; bench-verify the
F3 receive protocol and exact channel/mode semantics with motors disconnected;
measure camera identities, modes, timestamps, calibration, and trigger support;
then validate ARM64 resource use on a Pi 5. Physical flight requires a separate
safety review.

## Executed validation (2026-10-26, Intel macOS)

Commands actually executed:

```bash
.venv/bin/python -m pytest -q
# 57 passed in 12.70s (pre-migration baseline)

g++ -std=c++17 -Wall -Wextra -Wpedantic -Isrc/ece3162_core/include \
  -c src/ece3162_core/src/msp.cpp -o /tmp/ece3162-cpp-check/msp.o
g++ -std=c++17 -Wall -Wextra -Wpedantic -Isrc/ece3162_core/include \
  -c src/ece3162_core/src/trajectory_tracker.cpp \
  -o /tmp/ece3162-cpp-check/trajectory_tracker.o
# both compiled without diagnostics

PYTHONPATH=src/ece3162_mission .venv/bin/python -m pytest -q \
  src/ece3162_mission/test tests
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

docker compose run --rm ece3162 bash -lc \
  'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   colcon list --base-paths src && colcon test --base-paths src && \
   colcon test-result --verbose'
# ece3162_algorithms, ece3162_bringup, ece3162_core,
# ece3162_interfaces, ece3162_mission, ece3162_perception
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
