# Integration audit

## Repository findings

- Modern code is Python 3.11 (`pyproject.toml`) with NumPy, SciPy, Matplotlib,
  pytest, MuJoCo, OpenCV, and optional MediaPipe. `rospy` is not importable in
  the audited macOS environment.
- Existing, tested components include a dense 3-D `VoxelMap`, spherical
  obstacle inflation, 6/18/26-connected A*, continuous segment collision
  checking, collision-checked shortcut smoothing, time parameterization, and a
  trajectory interpolator. These were reused rather than copied.
- No IAPF or MATLAB sources exist in the current repository. IAPF is therefore
  not integrated and no APF result is claimed. RRT options in the old runner
  are placeholders.
- The existing MuJoCo path is a kinematic research simulation. The recovered
  `drone_simulator.py` is Python 2 code for synthetic UKF sensor data and ROS
  ground truth; it does not accept waypoints or model closed-loop tracking.
- The recovered ROS package is `archive/duckiedrone_pidrone_pkg/` (`pidrone_pkg` in
  `package.xml`). It includes an archived Python 3.7 ARM environment and two
  ARM PyTorch wheels. They are historical artifacts, not project dependencies.

## Consolidation decisions

- **Implemented and verified:** `ece3162` namespace, geometric A*, Kinodynamic
  A*, B-splines, EGO-inspired optimization, double-integrator dynamics,
  waypoint/time-indexed tracking, map inflation/collision checks, stereo data
  primitives, ray mapping, localization health, MSP framing, and PID primitive.
- **Reusable:** recovered MSP framing/channel constants and controller/fault
  semantics informed the native adapters and safety documentation.
- **Replaced:** ROS-coupled PID math and synthetic UKF sensor simulation are not
  active runtime dependencies; native deterministic components cover current
  simulation and protocol tests.
- **Archived/hardware-dependent:** original ROS nodes, custom messages,
  MultiWii serial class, estimator variants, camera/optical-flow nodes, and
  fault protector remain in `archive/duckiedrone_pidrone_pkg/` for provenance
  and physical validation. They are not installed as a second application.
- **Obsolete for the active runtime:** recovered web UI, networking scripts,
  Python 2 utilities, bundled Python 3.7 ARM environment, and ARM wheels.
- **Missing/blocked:** calibrated physical stereo, verified visual odometry,
  ROS/controller transforms, flight-controller firmware validation, and safe
  airborne failure behavior.

## Verified legacy command flow

| Topic | Type | Producer / consumer | Finding |
|---|---|---|---|
| `/pidrone/state` | `pidrone_pkg/State` | state selector → PID/fault protector | Header, PoseWithCovariance, TwistWithCovariance |
| `/pidrone/desired/pose/absolute` | `geometry_msgs/Pose` | external → PID | Absolute x/y/z copied into setpoint; no header |
| `/pidrone/desired/pose/relative` | `geometry_msgs/Pose` | external → PID | Offset from current position |
| `/pidrone/desired/twist` | `geometry_msgs/Twist` | external → PID | x/y/z velocity and yaw rate |
| `/pidrone/position_control` | `std_msgs/Bool` | shared | Enables 30 Hz outer x/y position loop |
| `/pidrone/fly_commands` | `pidrone_pkg/RC` | PID → flight node | roll, pitch, yaw, throttle raw command fields |
| `/pidrone/desired/mode` | `pidrone_pkg/Mode` | UI → flight/PID | strings `DISARMED`, `ARMED`, `FLYING` |
| `/pidrone/mode` | `pidrone_pkg/Mode` | flight node → PID | current flight mode |

`pid_controller.py` runs its inner loop at 100 Hz and outer position loop at
30 Hz. `flight_controller_node.py` runs at 60 Hz and opens `/dev/ttyUSB0`, then
`/dev/ttyUSB1`. `h2rMultiWii.py` configures 115200 baud and commands
`MSP_SET_RAW_RC`. This is MSP/Cleanflight-era integration, not MAVLink/MAVROS.

The mode graph in source is DISARMED ↔ ARMED → FLYING → DISARMED. The flight
node initializes with a disarm command and disarms in its final/error handlers.
The fault protector watches heartbeats, battery, acceleration, angular rate,
altitude, and camera pose; camera pose loss disables position control, while
other faults terminate the flight loop and its `finally` path sends disarm.
Whether that airborne response is safe is **UNVERIFIED** and is a hardware
blocker.

## Unverified or incomplete

- Physical axis alignment, pose units in every estimator, yaw semantics, state
  covariance health criteria, and camera-to-world calibration.
- Which recovered estimator was deployed and which ROS launch/network process
  remaps its output into `/pidrone/state`.
- Firmware/protocol compatibility, actual serial port, Pi model, calibration,
  actuator condition, and safe failsafe behavior.
- `package.xml` omits some dependencies used by messages/scripts and the code
  uses Python 2 syntax and unsafe legacy `yaml.load` calls. A matching Ubuntu
  16.04/ROS runtime has not been reconstructed.

Consequently no live ROS publisher was added. `DuckiedroneBridge` is a tested,
ROS-free safety gate and message mock. A ROS wrapper must wait for bench
verification of frames and failsafe behavior.
