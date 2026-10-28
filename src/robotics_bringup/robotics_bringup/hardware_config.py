"""Fail closed before starting any physical-device launch actions."""

from pathlib import Path
import math

import yaml
from launch.substitutions import LaunchConfiguration


def _params(config, node):
    entry = config.get(node, {})
    if not isinstance(entry, dict):
        raise ValueError(f"{node}: invalid node configuration")
    value = entry.get("ros__parameters", {})
    if not isinstance(value, dict):
        raise ValueError(f"{node}: missing ROS parameters")
    return value


def validate(context, *, cameras=False, telemetry=False, integrated=False):
    path = Path(LaunchConfiguration("config").perform(context))
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"invalid hardware config: {path}")
    if cameras:
        camera = _params(config, "uvc_stereo")
        perception = _params(config, "stereo_perception")
        left, right = (camera.get(f"{side}_device", "") for side in ("left", "right"))
        for label, device in (("left", left), ("right", right)):
            if not isinstance(device, str) or not device.startswith(("/dev/v4l/by-id/", "/dev/v4l/by-path/")) or "UNCONFIGURED" in device or not Path(device).exists():
                raise ValueError(f"{label} camera: persistent device path missing or unavailable: {device}")
        if Path(left).resolve() == Path(right).resolve():
            raise ValueError("left and right paths refer to the same camera")
        calibration = camera.get("calibration_file", "")
        if not calibration or "UNCONFIGURED" in calibration or not Path(calibration).is_file():
            raise ValueError("measured stereo calibration file is required")
        if calibration != perception.get("calibration_file") or perception.get("mode") != "external":
            raise ValueError("camera and perception calibration/mode disagree")
        if perception.get("mounting_verified") is not True:
            raise ValueError("measured camera mounting transform is required")
        for key in ("body_from_left_optical_translation", "body_from_left_optical_rotation_vector"):
            values = perception.get(key)
            if not isinstance(values, list) or len(values) != 3 or not all(
                    isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                    for value in values):
                raise ValueError(f"measured mounting parameter {key} requires three finite SI values")
        if camera.get("use_sim_time", False) or perception.get("use_sim_time", False):
            raise ValueError("physical camera receipt timestamps require the host ROS clock")
        if not isinstance(perception.get("max_pair_delta_s"), (int, float)) or not 0 < perception["max_pair_delta_s"] <= 0.1:
            raise ValueError("configurable stereo synchronization tolerance is required")
    if telemetry:
        controller = _params(config, "msp_telemetry")
        device = controller.get("serial_device", "")
        if controller.get("transport_enabled") is not True or controller.get("hardware_output_enabled") is not False:
            raise ValueError("read-only MSP transport must be explicitly enabled with output disabled")
        if controller.get("protocol") != "legacy_msp_v1_candidate":
            raise ValueError("MSP v1 candidate protocol must be selected explicitly")
        if not isinstance(device, str) or not device.startswith(("/dev/serial/by-id/", "/dev/serial/by-path/")) or "UNCONFIGURED" in device or not Path(device).exists():
            raise ValueError(f"persistent serial device missing or unavailable: {device}")
    if integrated:
        controller = _params(config, "msp_telemetry")
        autonomy = _params(config, "autonomy_pipeline")
        gate = _params(config, "physical_command_gate")
        if not all(isinstance(controller.get(key), str) and controller[key].strip() and "UNCONFIGURED" not in controller[key]
                   for key in ("controller_board", "firmware_family", "firmware_version")):
            raise ValueError("controller board and firmware identity must be recorded before integrated bring-up")
        if autonomy.get("system_mode") != "hardware_read_only" or autonomy.get("simulate_vehicle") is not False or autonomy.get("auto_start") is not False or autonomy.get("hardware_output_enabled") is not False:
            raise ValueError("integrated launch must be read-only with simulation and automatic mission disabled")
        if gate.get("hardware_output_enabled") is not False or gate.get("emergency_stop") is not True:
            raise ValueError("physical command gate must remain disabled with emergency stop active")
    return []
