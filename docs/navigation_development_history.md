# Phase 4 implementation checklist

- [x] Native ROS 2 path: kinodynamic A*, C++ voxel mapping and inflation, B-spline initialization and derivatives, EGO-inspired optimization, trajectory tracking, stereo depth, visual odometry, state estimation, and bounded simulation.
- [x] Navigation action, state/map subscriptions, timestamped trajectory publication and consumption, command publication, replanning, stale-input braking, and hardware-disabled launch.
- [x] Removed Python copies of the production kinodynamic planner, optimizer, B-spline, tracker, stereo-depth estimator, state estimator, double-integrator, MSP encoder, and unused ray-map adapters.
- [x] Validate commands at the simulation input boundary, including frame, stamp, finite acceleration, bounds, and replay handling; malformed or missing commands cannot reach the native dynamics step.
- [x] Remove uncalled Python PID and compatibility re-export modules.
- [x] Affected packages build; focused simulation command-gate check and software navigation checks pass.

## Python retained intentionally

- `planning/astar.py`, `planning/path_utils.py`, `planning/collision_checker.py`, `mapping/voxel_map.py`, and `mapping/obstacle_inflation.py` are called by `run_planning_sim.py`, `run_duckiedrone_sim.py`, `run_mujoco_sim.py`, `run_hand_sim.py`, offline visualization, and their tests. The geometric 6/18/26-connected A* is distinct from the native kinodynamic planner; none of these modules is imported by a ROS navigation node. Python voxel arrays represent offline worlds; live occupancy integration and collision queries use `robotics_core::navigation::VoxelMap`.
- `trajectory/time_parameterization.py` supplies stop-at-corner profiles for offline MuJoCo playback; the ROS controller uses native B-splines. `simulation/mujoco_runner.py` and `kinematic_follower.py` orchestrate MuJoCo and visualization. `simulation/waypoint_closed_loop.py` is a deterministic offline point-mass regression fixture, not the live double-integrator simulator.
- `integration/waypoint_manager.py` is an offline mission state machine. `integration/drone_bridge.py` is an inert guarded reference adapter. Hand tracking, calibration, frame conversion, configuration loading, and plotting remain Python tooling.

## Validation to date

- Dev Container build and `rosdep check` passed; six packages discovered and built with Colcon.
- Native targeted tests: 27 passed. Python tooling tests: 13 passed.
- After the command gate change, `robotics_core` and `robotics_tools` built; the software command-gate check passed. The broader static/dynamic/disconnect regression reported static success and safe disconnect braking. Its first dynamic run replanned but then failed on a visual-odometry jump that latched native localization loss; the native failsafe stopped the mission. A focused dynamic rerun succeeded with two trajectories, 0.124 m truth goal error, and no sampled collisions. The regression checker now requires dynamic mission success rather than accepting a terminal failure. Physical flight hardware was untouched.
