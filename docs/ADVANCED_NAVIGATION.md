# Advanced navigation

The implemented pipeline is:

```text
VoxelMap -> Kinodynamic A* -> EGO-inspired control-point optimization
         -> validated clamped B-spline -> feed-forward PD tracker
         -> delayed/noisy estimated state -> double-integrator dynamics
```

Kinodynamic A* searches `(position cell, quantized velocity)` states. Its seven
motion primitives are coasting and ± acceleration on each axis. Each primitive
uses the analytical double-integrator equations and sampled continuous
collision queries. Search respects velocity/acceleration, timeout, expansion,
goal-position, and terminal-velocity limits. The distance/max-speed heuristic
is admissible at weight 1 when elapsed time is the search cost, although state
quantization means the returned trajectory is a resolution-limited candidate.
The ROS 2 simulation uses weight 2 for bounded runtime and therefore makes
no optimality claim.

`EgoOptimizer` minimizes second-difference smoothness, deviation from
global guidance, occupied-voxel clearance penalties, and velocity and
acceleration feasibility penalties. This is an
ESDF-free native adaptation of the high-level EGO-Planner formulation. It does
not implement the reference package's rebound search or topology handling,
and must not be described as equivalent.
Every candidate is independently sampled along the curve for collision,
velocity, and acceleration validation. Invalid replacements are rejected.

The tracker evaluates desired position, velocity, and acceleration and applies
`a_cmd = a_d + Kp(p_d-p_hat) + Kd(v_d-v_hat)`, followed by acceleration
saturation. It monitors state age, validity, tracking error, settling timeout,
and cancellation. Simulation is a translational double integrator, not full
attitude, thrust, rotor, or aerodynamic dynamics.

The dynamic scenario replaces a trajectory only after a newly inserted voxel
obstacle is inflated and a new global/local solution validates. The active
navigation path is native C++ and runs inside the ROS 2 autonomy node.
