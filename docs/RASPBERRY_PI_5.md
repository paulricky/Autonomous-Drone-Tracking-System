# Raspberry Pi 5 deployment

Use [Phase 5 deployment and bench preparation](PHASE5_DEPLOYMENT.md) for the
native ARM64 build, persistent configuration, camera calibration, read-only
telemetry, startup, shutdown, and diagnostics procedures. The production
planning and perception path is C++17; Python remains for tooling. The Pi 5
image and physical hardware have not yet been validated.

The Pi 5 and two USB cameras require a measured power and thermal design. Do
not assume the original 5 V/3 A UBEC is sufficient. Account for camera power,
cooling, USB cabling, payload mass, center of gravity, rigid baseline,
vibration, serial connectivity, battery load, and reduced flight time.
