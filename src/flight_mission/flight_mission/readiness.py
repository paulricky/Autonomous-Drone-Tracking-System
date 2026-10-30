"""Explicit flight blockers; configuration assertions cannot certify hardware."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Check:
    condition: str
    result: str
    reason: str
    corrective_action: str


def evaluate(*, operator_authorized=False, telemetry_fresh=False,
             position_velocity_fresh=False, output_disabled=True):
    checks = [
        Check("operator_authorization", "PASS" if operator_authorized else "FAIL",
              "operator authorization recorded" if operator_authorized else "operator authorization absent",
              "Independent operator flight review after every other requirement is verified"),
        Check("physical_output_inhibited", "PASS" if output_disabled else "FAIL",
              "physical actuator backend absent; output disabled" if output_disabled else "unsafe output configuration observed",
              "Keep all hardware_output_enabled settings false and emergency_stop true for bench work"),
        Check("read_only_telemetry", "PASS" if telemetry_fresh else "FAIL",
              "fresh connected legacy status observed" if telemetry_fresh else "status missing, stale, disconnected or unsupported layout",
              "Identify firmware, run read-only telemetry; inspect status payload, poll span and sensor errors"),
        Check("position_velocity_available", "PASS" if position_velocity_fresh else "FAIL",
              "finite fresh map-frame position/velocity observed" if position_velocity_fresh else "valid physical position/velocity unavailable",
              "Calibrate stereo, measure camera-to-body transform; verify state rates and reference motion"),
    ]
    unknown = {
        "controller_identity_protocol": ("No controller-specific compatibility validation", "Capture API, variant, version, board replies and compare with exact firmware source/configurator"),
        "physical_control_mode": ("Supported setpoint interface and mode unverified", "Verify controller-supported control interface and mode acknowledgement"),
        "sensor_calibration_validation": ("Physical calibration, axes and timing unverified", "Measure stereo intrinsics/extrinsics, mounting, IMU SI units/axes/clock; validate against reference motion"),
        "localization_health": ("VO validity alone cannot establish flight localization health", "Validate drift, velocity, reset/loss handling and frame transforms against measured motion"),
        "attitude_angular_rate": ("VehicleState contains only position and velocity; IMU counts are unfused", "Provide verified attitude/rates, body-frame units and timestamped measurement fusion required by the selected physical interface"),
        "measurement_covariance": ("VO covariance unmeasured; raw IMU covariance unavailable", "Measure and validate covariance; do not treat a negative covariance sentinel as a valid covariance matrix"),
        "trajectory_frame_consistency": ("Software uses map; physical controller frame/yaw/origin mapping unverified", "Measure and verify map/body/controller conventions including yaw and reset behavior"),
        "command_transformation": ("Map acceleration/double-integrator output has no physical multirotor mapping", "Implement bounded controller-specific setpoint transformation only after supported interface verification"),
        "motor_actuator_configuration": ("Mixer, motor order/direction, ESCs and battery limits unknown", "Inspect firmware configuration and perform separate supervised propeller-free checks"),
        "controller_failsafe": ("No verified airborne response to serial/localization/RC loss", "Verify exact firmware failsafe and pilot interaction; separately supervise propeller-free fault tests"),
        "emergency_stop_manual_recovery": ("ROS output gate does not establish independent pilot recovery", "Verify independent receiver/pilot override and emergency control through exact firmware, including during landing"),
        "power_battery": ("Raw analog counts do not certify supply/battery condition", "Measure regulated voltage under camera/CPU load, battery/cell limits, USB stability and temperature"),
        "safe_environment": ("No operator confirmation of props removed or test area", "Physically remove propellers; secure aircraft, clear area, establish pilot access and appropriate power setup"),
        "controlled_landing": ("ROS interlocks implemented; physical landing adapter unavailable", "Identify and integrate verified built-in landing, then verify acknowledgement, progress, timeout and ground indication"),
        "grounded_disarming_shutdown": ("Authoritative landed/disarmed feedback unavailable", "Verify ground indication and disarm acknowledgement before preparing shutdown; never disconnect during landing"),
        "native_arm64_execution": ("Current report cannot attest native Pi build/execution", "Run native Ubuntu 24.04 ARM64 build on Pi and retain build-info, build log and runtime diagnostics"),
    }
    checks.extend(Check(name, "UNVERIFIED", reason, action) for name, (reason, action) in unknown.items())
    return {"status": "NOT READY", "flight_ready": False,
            "checks": [asdict(check) for check in checks]}
