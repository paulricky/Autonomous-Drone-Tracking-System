# Executed benchmarks

Phase 4 now retains per-frame timing/accuracy CSV hooks in `results/perception/`
and a native `perception_benchmark` executable. Those implementation smoke
observations are not a completed performance campaign. Comparative throughput,
resource use, drift, map accuracy and Raspberry Pi 5 measurements are deferred
to Phase 5; the Phase 3 figures below do not predict stereo performance.

## Native C++ Phase 3 measurements

Thirty warm-cache trials after two warmups ran on the same macOS host with
Python 3.11 and Apple clang `-O3 -DNDEBUG`. Raw distributions are in
`results/advanced_navigation/native_benchmark.json`; both languages used the
same free 0.25 m voxel map, inputs, limits, and control points.

| Component | Python median | C++ median | Speedup | Validation |
|---|---:|---:|---:|---|
| Kinodynamic A* | 41.106 ms | 0.101 ms | 407.35× | goal/dynamics pass |
| B-spline, 10,000 evaluations | 360.680 ms | 1.236 ms | 291.89× | derivative test passes |
| Collision, 10,000 point queries | 111.710 ms | 0.178 ms | 626.72× | occupancy semantics match |
| EGO-inspired optimizer | 16.563 ms | 1.246 ms | 13.30× | gradients/feasibility pass |
| Dynamic replanning | 13,824 ms (historical) | 5.944 ms algorithm / 6.262 ms integrated | not directly comparable | ROS success |

Microbenchmark ratios include Python interpreter and per-call object overhead;
they do not predict the full ROS pipeline or Raspberry Pi 5. C++
free/static/constrained closed-loop planner times were 0.100/0.481/17.188 ms,
with optimizer times 6.320/19.521/32.056 ms. All had zero modeled collisions
and zero sampled dynamic-limit violations. Peak RSS was not collected.

The dynamic C++ ROS run inserted the obstacle at simulation time 2 s, detected
that the remaining curve was affected, accepted one replacement, and issued
the first updated control reference 6.262 ms after map-change handling began.
Its final error was 0.044 m (RMS tracking error 0.131 m). The historical Python
13.824 s result used the earlier end-to-end scenario and therefore is shown for
context, not claimed as an equivalent-workload speedup.

At a requested 20 ms period, the free ROS run measured 20.166 ms mean and
52.298 ms maximum update intervals. Callback execution averaged 0.112 ms and
peaked at 10.918 ms. 239 intervals were greater than the exact 20 ms target;
this Docker/macOS run provides no hard real-time guarantee.


Measured on the macOS development machine on 2026-10-26. These are not
Raspberry Pi 5 or physical-flight results. Machine-readable files and figures
are under `results/advanced_navigation/`.

| Metric | Basic | Static obstacles | Dynamic replan |
|---|---:|---:|---:|
| Global planning (s) | 7.264 | 3.574 | 7.558 |
| Expanded states | 558 | 263 | 511 |
| Local optimization (s) | 0.255 | 2.482 | 5.028 |
| Global path (m) | 3.750 | 5.186 | 4.616 |
| Optimized curve (m) | 4.000 | 5.343 | 4.512 |
| Executed path (m) | 4.342 | 5.902 | 6.088 |
| RMS position error (m) | 0.0800 | 0.0929 | 0.0669 |
| Maximum position error (m) | 0.1939 | 0.3124 | 0.2090 |
| Final goal error (m) | 0.0413 | 0.0246 | 0.1343 |
| Simulated duration (s) | 7.44 | 10.80 | 10.64 |
| Minimum clearance (m) | N/A | 0.284 | 0.237 |
| Collision samples | 0 | 0 | 0 |
| Saturation fraction | 0.0188 | 0.0370 | 0.0000 |

The dynamic run performed one replan after obstacle insertion at simulation
time 2.0 s. Total measured replacement computation latency was 13.824 s. This
is too slow for online physical avoidance and is an explicit readiness blocker.

For the static CLI process, `/usr/bin/time -l` reported 25.50 s wall time,
11.86 s user CPU, and 2.40 s system CPU. macOS denied the resource query needed
for peak resident memory, so no memory number is reported. Perception throughput
and camera acquisition rates were not measured because no camera/calibration
data were available.
