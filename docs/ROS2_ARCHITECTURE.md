# ROS 2 Jazzy architecture

## Runtime boundary

The repository root is the single colcon workspace. Packages below `src/` with
a `package.xml` are ROS 2 packages; `src/robotics_tools/` remains the validated Python
algorithm and deterministic-simulation library during the staged native port.
The recovered ROS 1 package is provenance only and is never on the runtime
path.

| Package | Language | Responsibility |
|---|---|---|
| `flight_interfaces` | IDL | Navigation action and planner/hardware status |
| `robotics_core` | C++17 | Native planners, B-splines, tracking, dynamics, voxel mapping, read-only MSP transport and command gating |
| `stereo_perception` | C++17 | UVC/synthetic/recorded stereo, rectification, depth/clouds, PnP odometry and state estimation |
| `flight_mission` | Python | Low-rate mission state and cancellation/failure policy |
| `robotics_bringup` | Python launch | Installed simulation and read-only hardware configuration |
| `robotics_tools` | Python | Offline geometry, hand-goal tooling, and legacy simulation helpers |

The standalone Python kinodynamic planner, EGO-inspired optimizer, B-spline,
trajectory tracker, double-integrator, stereo-depth estimator, state estimator,
and MSP encoder have been removed after native replacements were connected.
The offline geometric A*, voxel map, waypoint simulation, and MuJoCo tools still
contain Python computation. They are outside the ROS 2 navigation launch and
are retained for deterministic simulation, visualization, and reference tests.

## Frames and units

All active metric interfaces use SI units. In the stereo odometry baseline,
`map` and `odom` coincide; no global drift correction is implemented. `base_link` is FLU
(x forward, y left, z up). Camera optical frames use REP 103 optical axes (x
right, y down, z forward). Physical `base_link` to camera and IMU transforms
are required configuration for physical operation. A missing camera mounting
transform prevents external perception startup. IMU axes and scale remain
unverified, so raw controller counts do not enter the estimator.

## Interfaces

| Name | Type | Owner | Rate / QoS | Validity |
|---|---|---|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | hardware monitor, MSP telemetry, and physical gate | 1 Hz, reliable | closed or stale until verified |
| `mission/status` | `std_msgs/String` | autonomy pipeline | on state change, reliable | informational only |
| `navigate_to_pose` | `flight_interfaces/action/NavigateToPose` | autonomy pipeline | on demand | stamped `map` goal, cancellable |
| `planning/status` | `flight_interfaces/msg/PlannerStatus` | autonomy pipeline | per plan | measured latency, explicit state |
| `flight_controller/status` | `flight_interfaces/msg/FlightControllerStatus` | hardware monitor or read-only MSP node, in separate launches | 1 Hz | command gate always closed |
| `flight_controller/raw_telemetry` | `flight_interfaces/msg/MspRawTelemetry` | read-only MSP node | 1 Hz | raw counts; availability flags per query |
| `planning/trajectory` | `flight_interfaces/msg/TimedTrajectory` | autonomy pipeline | on plan, transient local and reliable | absolute start stamp, SI samples |

Streaming imagery/clouds/odometry use `sensor_msgs` and `nav_msgs`. The existing
`VehicleState`, `ControlCommand` and `TimedTrajectory` interfaces remain the
navigation boundary. See [stereo implementation](STEREO_PERCEPTION.md) for the
Phase 4 data sources, transforms and QoS. No Python algorithm substitutes for a
native perception, mapping or localization stage in this launch.

## Safety ownership

The actual controller firmware, modes, RC mapping, and failsafe behavior still
require physical inspection. The read-only serial node sends only allowlisted
MSP v1 telemetry queries after explicit configuration. The common command gate
checks freshness, frame, range, replay, and physical health; the physical node
reports a closed gate and has no actuator transport. Arming, takeoff, motor
tests, and physical command publication are not implemented. Serial transport
software tests use virtual ports only.

## Launch

`diagnostics.launch.py` starts the inert hardware monitor and mission status
node. `uvc_stereo.launch.py` starts physical cameras and the existing stereo
estimator. `read_only_telemetry.launch.py` starts only the MSP probe.
`hardware_integration.launch.py` starts camera perception, telemetry, and
autonomy with `simulate_vehicle=false` and `auto_start=false`; it also reports
the closed physical command gate. It submits no navigation goal. See
[Phase 5 deployment](PHASE5_DEPLOYMENT.md) for required measured configuration.
# Native navigation simulation

`autonomy_sim.launch.py` starts the inert hardware monitor and two C++ ROS
nodes. `simulated_drone` subscribes `control/acceleration`, integrates bounded
dynamics at 50 Hz, and publishes separate `state/ground_truth` and delayed,
noisy `state/estimate` channels. `autonomy_pipeline` consumes only estimates,
plans in local ENU `map`, and publishes `planning/trajectory`,
`planning/status`, and `mission/status`.

`TimedTrajectory` has an absolute ROS header start stamp and relative
`time_from_start` samples. Position is metres, velocity m/s, and acceleration
m/s². Estimate and command streams use sensor/best-effort depth-one QoS;
trajectories are reliable and transient-local. A dynamic map change checks the
remaining curve and triggers replacement only when affected. Failed replacement
causes bounded braking, which is not a physical hover or landing guarantee.

`autonomy_pipeline` samples the published `TimedTrajectory` for tracking, and
brakes if the matching trajectory message is unavailable. It also rejects stale
localization, invalid map observations, expired references, and excessive
tracking error.

`navigate_to_pose` is a cancellable ROS 2 action. Set `auto_start:=false` when
missions will arrive only through the action server; the launch demo defaults
to its configured goal. Cancellation publishes a bounded braking reference,
returns a canceled action result, and leaves the server available for a later
goal.
