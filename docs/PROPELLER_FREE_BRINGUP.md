# Immediate Pi bench sequence

**Flight status: NOT READY.** No Pi, OV9281 pair or flight controller was attached
to the macOS development host. Native ARM64 execution and every physical flight
requirement remain unverified. Aim for device inventory, frames and read-only
telemetry in this session. All commands below keep actuator output disabled.

1. Physically remove all propellers. Secure the aircraft and inspect power,
   polarity, grounds, USB/UART voltage compatibility and connectors using the
   actual board documentation. Identify the board markings and receiver. Have
   the independent pilot/emergency controls accessible. Use an appropriate
   regulated Pi supply and cooling; measure its voltage under dual-camera/CPU
   load. Record battery cell count/voltage and controller power arrangements;
   do not infer them from raw MSP battery counts or USB power alone.

2. On the **Pi**, from the repository root, verify the platform and build:

   ```bash
   uname -sm
   cat /etc/os-release
   docker info --format '{{.Architecture}}'
   sudo apt-get update
   sudo apt-get install -y v4l-utils usbutils
   scripts/pi_deploy.sh build
   scripts/pi_deploy.sh start
   scripts/pi_deploy.sh preflight
   scripts/pi_deploy.sh diagnose
   ```

   Require Ubuntu 24.04, Linux `aarch64`, and an ARM64 Docker daemon/image.
   Build logs persist in `deployment_data/logs/build_*.log`, image/source/host
   identity in `deployment_data/build-info.txt`. A successful AMD64 build is
   not native Pi evidence. Preflight must currently return `success=False`
   and `NOT READY`, listing each condition and corrective action.

3. Identify both OV9281 **capture** interfaces and serial device without writing
   controller configuration:

   ```bash
   v4l2-ctl --list-devices
   ls -l /dev/v4l/by-id /dev/v4l/by-path
   ls -l /dev/serial/by-id /dev/serial/by-path
   read -rp 'Persistent left capture path: ' LEFT_CAMERA_DEVICE
   read -rp 'Persistent right capture path: ' RIGHT_CAMERA_DEVICE
   read -rp 'Persistent controller serial path: ' FLIGHT_CONTROLLER_DEVICE
   export LEFT_CAMERA_DEVICE RIGHT_CAMERA_DEVICE FLIGHT_CONTROLLER_DEVICE
   v4l2-ctl -d "$LEFT_CAMERA_DEVICE" --all --list-formats-ext
   v4l2-ctl -d "$RIGHT_CAMERA_DEVICE" --all --list-formats-ext
   ```

   Select distinct video capture nodes, not metadata interfaces. Identical
   camera USB serials require fixed USB-port `by-path` identities. Keep those
   ports and left/right assignment fixed. Confirm selected modes on **both**
   cameras; defaults in hardware.yaml are provisional, not measured support.

4. Capture stationary checkerboard pairs at varied distances, positions and
   tilts. Supply measured board dimensions and supported capture settings:

   ```bash
   read -rp 'Width height FPS maximum_receipt_skew_ms pair_count: ' WIDTH HEIGHT FPS DELTA_MS COUNT
   read -rp 'Supported four-character pixel format (blank for driver default): ' FOURCC
   scripts/pi_deploy.sh capture "$WIDTH" "$HEIGHT" "$FPS" "$DELTA_MS" "$COUNT" bench_board "$FOURCC"
   read -rp 'Inner columns inner rows measured_square_size_m: ' COLS ROWS SQUARE_M
   scripts/pi_deploy.sh calibrate bench_board "$COLS" "$ROWS" "$SQUARE_M"
   ```

   Use a new capture name on repeat runs; existing images are preserved.
   Inspect images, manifest timing and `stereo.yaml` quality/identities.
   Calibration needs at least eight accepted pairs. Inspect supported exposure
   controls and motion blur. Receipt skew is **not exposure synchronization**:
   these independent USB cameras have no verified hardware sync. Keep the
   board stationary for each pair; validate depth and scale against measured
   targets before using stereo measurements for navigation.

5. Edit `deployment_data/config/hardware.yaml`. Enter measured device paths,
   width/height/FPS/FourCC and both calibration paths, e.g.
   `/var/lib/robotics/calibration/bench_board/stereo.yaml`. Measure baseline
   and rigid camera mounting. `body_from_left_optical_translation` is metres
   expressed in the body frame; the rotation vector is in radians and maps
   left optical coordinates into the body frame. Set `mounting_verified`
   only after verifying this transformation against physical axes and motion.
   Set `max_pair_delta_s` from measured timing limits; tighter receipt pairing
   cannot establish simultaneous exposure. Keep all output settings false,
   `operator_authorized: false`, and `emergency_stop: true`.

6. Stop the diagnostics-only container before hardware launches to avoid
   duplicate mission/preflight service providers. Identify firmware using board
   markings and its appropriate read-only configurator/version display first.
   Record firmware family, exact version, target, port, configured baud and
   supported protocol. Do not guess wiring, baud, or flash new firmware.
   Recovered architecture names SP Racing F3 while old IMU comments name Naze32;
   neither establishes the identity or IMU model of the aircraft on the bench.
   If MSP v1 framing is appropriate, set `transport_enabled: true`,
   `protocol: msp_v1_read_only_candidate`, `serial_device`, and measured
   `baud_rate` (implemented transport supports 115200 or 57600 only).

   ```bash
   scripts/pi_deploy.sh stop
   scripts/pi_deploy.sh telemetry
   # In another terminal on the Pi:
   scripts/pi_deploy.sh bench-shell
   ros2 topic echo /flight_controller/identity --once --qos-durability transient_local
   ros2 topic echo /flight_controller/raw_telemetry --once
   ros2 topic echo /flight_controller/status --once
   ros2 service call /preflight/check std_srvs/srv/Trigger '{}'
   ```

   Probe order is API_VERSION, then variant/version/board for supported API
   major 1/protocol byte 0; legacy IDENT is a fallback. Unknown API versions
   inhibit further queries. Raw extended status/analog/version/board payloads
   are preserved, not assigned guessed firmware semantics. Exact legacy
   decoders may report unavailable on extended layouts. Missing replies mean
   check port/protocol/wiring/permissions and close any competing configurator.
   No MSP v2 parser, physical command compatibility, armed/landed indication,
   landing mode or failsafe behavior is certified by this probe.

7. End telemetry with Ctrl-C, then run camera/localization plus telemetry:

   ```bash
   scripts/pi_deploy.sh sensors
   # Another Pi terminal:
   scripts/pi_deploy.sh bench-shell
   ros2 topic hz /camera/left/image_raw
   ros2 topic hz /camera/right/image_raw
   ros2 topic echo /perception/diagnostics --once
   ros2 topic echo /visual_odometry --once
   ros2 topic echo /perception/state_estimate --once
   ros2 run tf2_ros tf2_echo base_link left_camera_optical_frame
   ros2 run tf2_ros tf2_echo map base_link
   ros2 service call /preflight/check std_srvs/srv/Trigger '{}'
   ```

   Rate commands run until Ctrl-C. Check image/CameraInfo stamps match, clock
   does not jump, timestamps increase, frame pairing/age rejections are
   explained, and stationary pose/velocity remain stable. Move the **unpowered
   propulsion** aircraft by known distances and rotations to verify scale,
   axes and map/body transforms. State contains map position and velocity;
   odometry includes VO orientation but no fused IMU angular rate. Negative
   covariance sentinel is **not valid covariance for flight estimation**.
   MSP raw IMU counts have unverified scale, axis order, acquisition time,
   extrinsics, gravity convention and covariance. Measure these against exact
   firmware source and reference movements before adding SI conversion/fusion.
   Telemetry is a 1 Hz inventory poll, too slow to guarantee the physical gate's
   0.25 s freshness requirement. Poll completion is not acquisition time;
   `poll_started`/`poll_duration_s` expose the aggregate sampling interval.

8. Exercise the **non-actuating** mission paths in that bench shell:

   ```bash
   ros2 service call /mission/land std_srvs/srv/Trigger '{}'
   ros2 topic echo /mission/landing_status --once
   ros2 topic echo /mission/navigation_inhibited --once --qos-durability transient_local
   ros2 service call /mission/disarm std_srvs/srv/Trigger '{}'
   ros2 service call /mission/prepare_shutdown std_srvs/srv/Trigger '{}'
   ```

   Expected: LAND latches navigation inhibition and returns
   `landing_failed` because the physical controller adapter is unavailable.
   Repeated requests never restart navigation. Disarm and shutdown preparation
   return false. No controller mode change, RC override, arm/disarm, motor or
   landing command is transmitted. The service runs on the Pi independently
   of gestures and the external laptop process; it does not establish a
   physical operator input when the laptop is unavailable. Independent
   receiver/pilot recovery remains a separate mandatory hardware gate.
   Restarting processes is not an airborne recovery procedure.

9. Collect evidence from another Pi terminal **before** ending the foreground
   launch (its temporary container is removed at exit):

   ```bash
   scripts/pi_deploy.sh diagnose
   docker logs robotics-bench > deployment_data/logs/bench_console.log 2>&1
   docker exec robotics-bench bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/robotics/install/setup.bash && timeout 10s ros2 service call /preflight/check std_srvs/srv/Trigger "{}"' > deployment_data/logs/preflight.txt
   ```

   ROS logs persist in `deployment_data/logs/`; diagnostics include USB identity,
   V4L2 modes, kernel errors, thermal/throttling information when available,
   serial permissions and both container logs. A permission-denied power or
   kernel diagnostic is missing evidence, never a healthy result. For USB
   re-enumeration, end the bench launch and restart it to recreate Docker
   device mappings; verify reconnection on each camera and controller with
   propulsion unpowered. Do not test airborne loss responses here.

## Landing, recovery and shutdown gates before any flight

ROS implements LAND_REQUESTED → LANDING (only on mode acknowledgement) →
LANDED (authoritative landed/verified ground indication) → DISARMED (fresh
controller confirmation), plus latched LANDING_FAILED. The tested controller
contract handles rejection, stale/replayed feedback, communication/localization
loss, pilot takeover and timeouts without guessing throttle or disarming in
air. Only an independently verified firmware adapter may apply its documented
fault policy. The production backend is unavailable; no aircraft descent or
physical landing has been demonstrated. The native navigation double integrator
does not model multirotor landing. Mocked controller descent tests exercise
state transitions only; real firmware SITL and aircraft tests remain required.

Before flight: identify and integrate the exact supported controlled-landing
interface; verify mode acknowledgement, progress/timeout monitoring, ground
detection, grounded-only disarming and receiver/pilot override interaction.
Verify command mapping, calibration/covariance, localization, motor configuration,
power limits and each controller failsafe independently. No readiness setting
can currently enable physical output.

After a future verified landing, require fresh confirmed grounded **and**
disarmed telemetry, then an explicit Pi-local operator authorization
(`shutdown_authorized` is false by default) and a successful
`/mission/prepare_shutdown` response. The flag is a software interlock;
ROS 2 parameter access alone does not authenticate an operator, so access
control must be validated separately before flight.
The operator must then secure propulsion power using the controller-specific
procedure, collect logs, stop ROS, and finally shut down the Pi normally.
The query itself never shuts down the OS or disconnects the controller.
Never stop services, cut Pi/controller power, or disconnect serial during
active landing as a normal shutdown step. The current unavailable adapter
cannot authorize this workflow; manual recovery and confirmation must be
established independently before any flight.

Any later motor check is a **separate supervised procedure** requiring physical
confirmation of removed propellers, the identified firmware's documented
motor-test safeguards, known mixer/motor mapping and accessible independent
emergency controls. No motor test is part of this sequence or automated tests.
