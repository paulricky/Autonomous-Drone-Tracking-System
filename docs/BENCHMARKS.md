# Executed benchmarks

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
