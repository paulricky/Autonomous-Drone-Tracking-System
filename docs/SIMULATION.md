# Duckiedrone software-only simulation

Install the existing project environment, then run:

```bash
python -m pytest -q
ros2 launch robotics_bringup autonomy_sim.launch.py scenario:=free
ros2 launch robotics_bringup autonomy_sim.launch.py scenario:=static
ros2 launch robotics_bringup autonomy_sim.launch.py scenario:=dynamic
MPLBACKEND=Agg python scripts/run_duckiedrone_sim.py --scenario baseline
MPLBACKEND=Agg python scripts/run_duckiedrone_sim.py --scenario obstacle
```

Use `--output-dir PATH` to change results and `--show` for interactive plots.
Defaults are in `config/duckiedrone_simulation.yaml`; hardware publication is
false and the runner rejects a configuration that enables it.

The loop is A* → collision-checked shortened path → waypoint state machine →
proportional velocity demand → speed and acceleration limiting → integrated
position → fresh feedback. It has nonzero transient error and never teleports
the state to a waypoint. It is a simplified point-mass integration test, not a
quadcopter attitude, rotor, aerodynamics, or flight-safety validation.

The baseline world is obstacle-free at constant 1 m altitude. The obstacle
world reuses the repository's 3-D multi-obstacle world. Reported clearance is
the distance from the simulated vehicle center to obstacle boxes minus the
configured spherical drone radius; collision count is the number of sampled
states intersecting that physical envelope. Planning uses the additionally
inflated map (drone radius plus safety margin).

Tracking error is the instantaneous distance to the currently active waypoint,
so its maximum includes the initial distance to a newly issued waypoint. A
world with no obstacles reports minimum clearance as `null` (not applicable).

IAPF is unavailable because no IAPF/MATLAB implementation is present. The
runner intentionally offers only the verified A* option.
# Native closed-loop scenarios

Run `ros2 launch robotics_bringup autonomy_sim.launch.py scenario:=<name>` after
sourcing the Jazzy workspace. State evolves with the double-integrator
equations and bounded acceleration/velocity; it is never assigned to a desired
point. The tracker uses `state/estimate`, while truth is evaluation-only.
`dynamic` inserts an obstacle at two seconds and revalidates the remaining
curve before replanning.

The installed simulation entry point is `autonomy_sim_node`; the standalone
navigation demonstration has been removed in favor of the ROS 2 launch.

Failure regression coverage includes unreachable planning, optimizer rejection,
stale localization, excessive tracking error, trajectory settling timeout, and
action cancellation. The action smoke test is `python3
scripts/test_navigation_action.py` while `autonomy_sim_node` runs with
`auto_start:=false`.

# Phase 4 native perception navigation

`ros2 launch robotics_bringup perception_navigation.launch.py` runs the native
stereo-to-navigation chain. Synthetic images are rendered from fixed world
geometry and moving camera pose; depth and motion are reconstructed by the
actual stereo/PnP algorithms. The planner consumes the resulting voxel map,
and tracking consumes `perception/state_estimate`. The low-rate Python mission
manager only handles readiness and the navigation action.

See [stereo implementation](STEREO_PERCEPTION.md) for the four launch modes,
dynamic/sensor-loss arguments and assumptions. The bounded synthetic test volume
explicitly allows unknown cells; conservative unknown handling remains the
default for other perception-map runs. Comprehensive validation is deferred to
Phase 5. Historical reference simulations above remain available.
