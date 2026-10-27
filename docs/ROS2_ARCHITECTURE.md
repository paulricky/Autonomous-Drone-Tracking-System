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
| `robotics_core` | C++17 | Native planners, B-splines, tracking, dynamics, voxel mapping, inert hardware diagnostics |
| `stereo_perception` | C++17 | UVC/synthetic/recorded stereo, rectification, depth/clouds, PnP odometry and state estimation |
| `flight_mission` | Python | Low-rate mission state and cancellation/failure policy |
| `robotics_bringup` | Python launch | Installed parameters and hardware-disabled launch |
| `robotics_tools` | Python | Offline geometry, hand-goal tooling, and legacy simulation helpers |

The standalone Python kinodynamic planner, EGO-inspired optimizer, B-spline,
trajectory tracker, double-integrator, stereo-depth estimator, state estimator,
and MSP encoder have been removed after native replacements were connected.
The offline geometric A*, voxel map, waypoint simulation, and MuJoCo tools still
contain Python computation. They are outside the ROS 2 navigation launch and
remain migration work before the native port can be called complete.

## Frames and units

All active metric interfaces use SI units. In the stereo odometry baseline,
`map` and `odom` coincide; no global drift correction is implemented. `base_link` is FLU
(x forward, y left, z up). Camera optical frames use REP 103 optical axes (x
right, y down, z forward). Physical `base_link` to camera and IMU transforms
are intentionally absent until measured. A missing critical transform keeps
the hardware command gate closed.

## Interfaces

| Name | Type | Owner | Rate / QoS | Validity |
|---|---|---|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | hardware monitor and perception | 1 Hz, reliable | `STALE` while disconnected |
| `mission/status` | `std_msgs/String` | autonomy pipeline | on state change, reliable | informational only |
| `navigate_to_pose` | `flight_interfaces/action/NavigateToPose` | autonomy pipeline | on demand | stamped `map` goal, cancellable |
| `planning/status` | `flight_interfaces/msg/PlannerStatus` | autonomy pipeline | per plan | measured latency, explicit state |
| `flight_controller/status` | `flight_interfaces/msg/FlightControllerStatus` | hardware monitor | 1 Hz | explicitly disconnected, command gate closed |
| `planning/trajectory` | `flight_interfaces/msg/TimedTrajectory` | autonomy pipeline | on plan, transient local and reliable | absolute start stamp, SI samples |

Streaming imagery/clouds/odometry use `sensor_msgs` and `nav_msgs`. The existing
`VehicleState`, `ControlCommand` and `TimedTrajectory` interfaces remain the
navigation boundary. See [stereo implementation](STEREO_PERCEPTION.md) for the
Phase 4 data sources, transforms and QoS. No Python algorithm substitutes for a
native perception, mapping or localization stage in this launch.

## Safety ownership

The SP Racing F3 retains low-level stabilization. Linux may eventually send
only a verified Cleanflight/MSP reference mode. The current lifecycle monitor
does not open a serial device and refuses configuration when
`hardware_output_enabled` is true. Arming, takeoff, motor tests, and physical
command publication are not implemented. Simulation and tests cannot access a
serial transport.

## Launch

`ros2 launch robotics_bringup diagnostics.launch.py` starts only the inert
hardware monitor and mission status node. Lifecycle activation is deliberate,
not automatic. Future full-system launch descriptions must retain the same
false-by-default hardware parameter.
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
