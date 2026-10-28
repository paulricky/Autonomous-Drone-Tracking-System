# Hardware readiness checklist

The implementation and operator procedures are in
[Pi deployment and bench preparation](PI_DEPLOYMENT.md). The
read-only MSP probe and physical command gate remain non-actuating.

Phase 4 UVC acquisition supports configured persistent Linux video-device paths,
with OpenCV/V4L2 host-reception timestamps. No OV9281 cameras or Raspberry Pi 5
were available for testing. Real stereo calibration, mounting extrinsics, dual
USB bandwidth/power, exposure timing, disconnect behavior and synchronization
remain unverified. Synthetic rig parameters must not be treated as physical
calibration. The camera-only launch does not start navigation or flight output.

Live autonomous flight is blocked. Complete this checklist with propellers off
and an accessible physical kill/disarm control before considering tethered or
restrained testing.

- Preserve the full SD image and recovered source archive read-only; record hashes.
- Identify the exact Raspberry Pi model, power budget, OS image, Python version,
  ROS distribution, and all catkin/system dependencies.
- Inventory persistent serial-device paths, USB adapter identity,
  permissions/group membership, candidate 115200-baud stability, and the
  actual board and firmware configuration.
- Confirm MSP variant and `MSP_SET_RAW_RC` channel order/ranges against firmware.
- Back up flight-controller settings; verify port mapping, receiver/failsafe,
  mixer, motor order/direction, ESC condition, and sensor calibration.
- Inspect frame, wiring, connectors, UBEC 5 V regulation under load, battery,
  motors, ESCs, camera cable, and camera initialization.
- Verify every state-estimator axis, sign, unit, timestamp, covariance, update
  rate, map frame, origin, and camera extrinsic with motion capture/reference.
- Bench-test ROS topic types/rates and determine which estimator owns
  `/pidrone/state`; resolve legacy `Body` versus `global` frame labels.
- Characterize communication loss and every fault-protector transition. Decide
  a validated airborne response for localization loss; stopping publication,
  neutral/hold, landing, flight-controller failsafe, and disarm are different.
- Confirm physical kill access, no automatic arming, startup in DISARMED,
  command watchdog behavior, and recovery after process/serial/power loss.
- With propellers removed, validate RC channel direction/limits, arming logic,
  throttle cutoff, and motor mapping. Do not infer these from simulation.
- Only after independent safety review: restrained low-energy manual tests,
  then state-estimation/position-hold tests, before autonomous waypoints.

Required before enabling a ROS wrapper: verified transform into the legacy
controller frame, fresh/healthy state criteria, command rate/limits, operator
authorization, preflight result, tested fault response, and an explicit
hardware-enable setting. The current bridge cannot arm, take off, hold, land,
or disarm and is intentionally inactive.
