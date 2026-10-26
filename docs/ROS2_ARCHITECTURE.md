# ROS 2 Jazzy architecture

## Runtime boundary

The repository root is the single colcon workspace. Packages below `src/` with
a `package.xml` are ROS 2 packages; `src/ece3162/` remains the validated Python
algorithm and deterministic-simulation library during the staged native port.
The recovered ROS 1 package is provenance only and is never on the runtime
path.

| Package | Language | Responsibility |
|---|---|---|
| `ece3162_interfaces` | IDL | Navigation action and planner/hardware status |
| `ece3162_core` | C++17 | ROS-independent MSP and tracking libraries; inert hardware diagnostics |
| `ece3162_perception` | C++17 | Bounded stereo timestamp pairing and synchronization diagnostics |
| `ece3162_mission` | Python | Low-rate mission state and cancellation/failure policy |
| `ece3162_bringup` | Python launch | Installed parameters and hardware-disabled launch |
| `ece3162_algorithms` | Python | Existing planning, perception, mapping, simulation and benchmarks |

## Frames and units

All active metric interfaces use SI units. `map` is a drift-corrected global
ENU frame, `odom` is a locally continuous ENU frame, and `base_link` is FLU
(x forward, y left, z up). Camera optical frames use REP 103 optical axes (x
right, y down, z forward). Physical `base_link` to camera and IMU transforms
are intentionally absent until measured. A missing critical transform keeps
the hardware command gate closed.

## Interfaces

| Name | Type | Owner | Rate / QoS | Validity |
|---|---|---|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | hardware monitor and future subsystems | 1 Hz, reliable | `STALE` while disconnected |
| `mission/status` | `std_msgs/String` | mission manager | 1 Hz, reliable | informational only |
| `NavigateToPose` | `ece3162_interfaces/action/NavigateToPose` | future mission action server | on demand | stamped `map` goal, cancellable |
| `PlannerStatus` | `ece3162_interfaces/msg/PlannerStatus` | future planning node | per plan | measured latency, explicit state |
| `FlightControllerStatus` | `ece3162_interfaces/msg/FlightControllerStatus` | future MSP adapter | 1–10 Hz | reports gate independently of link |

Streaming sensor and state data use standard `sensor_msgs`, `nav_msgs`, and
`geometry_msgs`. No custom image, point-cloud, pose, path, or trajectory type
is introduced.

## Safety ownership

The SP Racing F3 retains low-level stabilization. Linux may eventually send
only a verified Cleanflight/MSP reference mode. The current lifecycle monitor
does not open a serial device and refuses configuration when
`hardware_output_enabled` is true. Arming, takeoff, motor tests, and physical
command publication are not implemented. Simulation and tests cannot access a
serial transport.

## Launch

`ros2 launch ece3162_bringup diagnostics.launch.py` starts only the inert
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

`navigate_to_pose` is a cancellable ROS 2 action. Set `auto_start:=false` when
missions will arrive only through the action server; the launch demo defaults
to its configured goal. Cancellation publishes a bounded braking reference,
returns a canceled action result, and leaves the server available for a later
goal.
