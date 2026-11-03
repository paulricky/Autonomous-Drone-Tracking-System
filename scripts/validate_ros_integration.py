#!/usr/bin/env python3
"""ROS observer and simulation fault injector; never publishes navigation inputs.

Run inside the sourced Jazzy workspace. Starts one launch per scenario, waits on
messages, records actual ground truth for evaluation, and always reaps children.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import resource
import signal
import subprocess
import time

import numpy as np

GOAL_EXECUTING = 2
GOAL_ABORTED = 6


def distribution(values):
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), mean=float(a.mean()), median=float(np.median(a)),
                std=float(a.std()), p95=float(np.percentile(a, 95)), max=float(a.max()))


def stamp(message):
    t = message.header.stamp
    return t.sec + t.nanosec * 1e-9


def vector(p):
    return np.array([p.x, p.y, p.z])


def navigation_ready(data, now):
    """Require observed, fresh sensors and actual controlled motion before disconnecting."""
    return bool(
        len(data["images"]) >= 5 and len(data["maps"]) >= 3
        and sum(e[2] and not e[3] for e in data["estimates"]) >= 5
        and sum(not c[3] for c in data["controls"]) >= 5
        and data["trajectories"] and data["truth"]
        and not data["controls"][-1][3]
        and data["controls"][-1][2] == data["trajectories"][-1].trajectory_id
        and data["estimates"][-1][2] and not data["estimates"][-1][3]
        and all(0 <= now - t <= .5 for t in (
            data["images"][-1], data["maps"][-1][0], data["estimates"][-1][0],
            data["controls"][-1][0], data["truth"][-1][0]))
        and np.linalg.norm(data["truth"][-1][1] - data["truth"][0][1]) >= .1
        and np.linalg.norm(data["truth"][-1][2]) >= .1
        and data["action_states"] and data["action_states"][-1][2] == GOAL_EXECUTING)


def disconnect_safety(fault, terminal, data):
    """A specific injected sensor loss must abort, brake, and remain stopped."""
    detected = fault.get("detected_at")
    failures = [t for t, detail in data["failures"] if detected is not None
                and t >= detected and detail == "localization or map unavailable"]
    aborted = failures[0] if failures else None
    after = [s for s in data["truth"] if aborted is not None and s[0] >= aborted]
    hold = [s for s in after if s[0] >= aborted + 1.0]
    commands = [c for c in data["controls"] if aborted is not None and c[0] >= aborted]
    displacement = float(np.linalg.norm(after[-1][1] - after[0][1])) if after else None
    speed = float(np.linalg.norm(after[0][2])) if after else 0.0
    # SimulatorNode brakes at 2 m/s^2; allow its 150 ms command watchdog plus sampling.
    stopping_limit = speed * speed / 4.0 + .15 * speed + .05
    checks = dict(
        healthy_before_fault=bool(fault.get("healthy")),
        injection_acknowledged=bool(fault.get("acknowledged")),
        disconnect_reported=bool(detected is not None and detected >= fault.get("requested_at", math.inf)),
        expected_abort=terminal == "failed:localization or map unavailable" and aborted is not None,
        bounded_detection=aborted is not None and 0 <= aborted - detected <= 1.0,
        action_aborted=bool(detected is not None and any(
            t >= detected and goal == fault.get("goal_id") and state == GOAL_ABORTED
            for t, goal, state in data["action_states"])),
        sensors_stopped=bool(detected is not None and all(
            t <= detected for t in data["images"] + [m[0] for m in data["maps"]]
            + [e[0] for e in data["estimates"]])),
        stale_inputs_detected=bool(aborted is not None and data["maps"] and data["estimates"]
            and aborted - min(data["maps"][-1][0], data["estimates"][-1][0]) >= .48),
        braking_only=bool(commands and all(c[3] and np.all(np.isfinite(c[1]))
            and np.linalg.norm(c[1]) <= 2.0 + 1e-6 for c in commands)),
        no_new_trajectory=bool(aborted is not None and not any(
            stamp(tr) >= aborted for tr in data["trajectories"])),
        bounded_stopping_distance=displacement is not None and displacement <= stopping_limit,
        sustained_stop=bool(len(hold) >= 10 and hold[-1][0] - hold[0][0] >= .5
            and all(np.linalg.norm(s[2]) <= .05 for s in hold)
            and all(np.linalg.norm(s[1] - hold[0][1]) <= .03 for s in hold)),
    )
    return dict(checks=checks, passed=all(checks.values()), detected_at=detected,
                aborted_at=aborted, stopping_distance_m=displacement,
                stopping_distance_limit_m=stopping_limit,
                hold_duration_s=hold[-1][0] - hold[0][0] if hold else 0.0)


def run(scenario, root):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from action_msgs.msg import GoalStatusArray
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from std_msgs.msg import String
    from sensor_msgs.msg import Image, PointCloud2
    from flight_interfaces.msg import VehicleState, TimedTrajectory, ControlCommand, PlannerStatus

    output = root / scenario
    output.mkdir(parents=True, exist_ok=True)
    integrated = scenario in ("static", "dynamic", "disconnect", "localization_loss")
    launch = "perception_navigation" if integrated else {
        "synthetic": "synthetic_perception", "localization": "stereo_localization",
        "uvc_unconfigured": "uvc_stereo"}[scenario]
    command = ["ros2", "launch", "robotics_bringup", launch + ".launch.py"]
    if integrated:
        command += [f"results_directory:={output}"]
    if scenario == "dynamic":
        command += ["dynamic_after_s:=3.0"]
    elif scenario == "localization_loss":
        command += ["low_texture_after_frames:=30"]
    node = rclpy.create_node("ros_integration_evaluator_" + scenario)
    status, truth, estimates, controls, images, maps, trajectories = [], [], [], [], [], [], []
    failures, action_states = [], []
    fault, fault_future = {}, None
    data = dict(truth=truth, estimates=estimates, controls=controls, images=images,
                maps=maps, trajectories=trajectories, failures=failures, action_states=action_states)
    fault_client = AsyncParameterClient(node, "/stereo_perception") if scenario == "disconnect" else None

    def on_diagnostic(message):
        if "requested_at" not in fault:
            return
        for diagnostic in message.status:
            if (diagnostic.name == "stereo_perception" and diagnostic.hardware_id == "synthetic"
                    and diagnostic.level == DiagnosticStatus.ERROR and diagnostic.message == "camera disconnected"):
                fault.setdefault("detected_at", stamp(message))

    def on_action_status(message):
        now = node.get_clock().now().nanoseconds * 1e-9
        action_states.extend((now, bytes(s.goal_info.goal_id.uuid).hex(), s.status)
                             for s in message.status_list)

    subs = [
        node.create_subscription(String, "mission/status", lambda m: status.append(m.data), 10),
        node.create_subscription(VehicleState, "state/ground_truth", lambda m: truth.append((stamp(m), vector(m.position), vector(m.velocity))), qos_profile_sensor_data),
        node.create_subscription(VehicleState, "perception/state_estimate", lambda m: estimates.append((stamp(m), vector(m.position), m.valid, m.ground_truth)), qos_profile_sensor_data),
        node.create_subscription(ControlCommand, "control/acceleration", lambda m: controls.append((stamp(m), vector(m.acceleration), m.trajectory_id, m.braking)), qos_profile_sensor_data),
        node.create_subscription(Image, "stereo/left/image_raw", lambda m: images.append(stamp(m)), qos_profile_sensor_data),
        node.create_subscription(PointCloud2, "mapping/occupied_voxels", lambda m: maps.append((stamp(m), m.width*m.height)), qos_profile_sensor_data),
        node.create_subscription(TimedTrajectory, "planning/trajectory", lambda m: trajectories.append(m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE)),
        node.create_subscription(DiagnosticArray, "perception/diagnostics", on_diagnostic, 10),
        node.create_subscription(PlannerStatus, "planning/status", lambda m: failures.append(
            (m.stamp.sec + m.stamp.nanosec * 1e-9, m.detail)) if m.state == m.FAILED else None, 10),
        node.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", on_action_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE)),
    ]
    usage_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    begun = time.monotonic()
    with (output / "launch.log").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        limit = 55 if integrated else 7
        try:
            while process.poll() is None and time.monotonic() - begun < limit:
                rclpy.spin_once(node, timeout_sec=.1)
                now = node.get_clock().now().nanoseconds * 1e-9
                if (fault_client is not None and fault_future is None
                        and fault_client.services_are_ready() and navigation_ready(data, now)):
                    goal = action_states[-1][1]
                    fault.update(healthy=True, requested_at=now, goal_id=goal,
                                 healthy_images=len(images), healthy_maps=len(maps))
                    # The existing simulation parameter disconnects on the next camera tick.
                    fault_future = fault_client.set_parameters([Parameter("disconnect_after_frames", value=0)])
                if fault_future is not None and fault_future.done() and "acknowledged" not in fault:
                    response = fault_future.result()
                    fault["acknowledged"] = bool(response and len(response.results) == 1
                                                 and response.results[0].successful)
            timed_out = process.poll() is None
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            # Receive final reliable abort/status messages already queued at process exit.
            drain_until = time.monotonic() + .2
            while time.monotonic() < drain_until:
                rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
    wall = time.monotonic() - begun
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    log = (output / "launch.log").read_text()
    terminal = next((s for s in reversed(status) if s == "succeeded" or s.startswith("failed:")), None)
    if terminal is None:
        matches = re.findall(r"mission_status=(succeeded|failed:[^\n]+)", log)
        terminal = matches[-1] if matches else None
    errors, distances, collisions, clearances = [], [], 0, []
    for i, (t, p, velocity) in enumerate(truth):
        if i:
            distances.append(float(np.linalg.norm(p-truth[i-1][1])))
        boxes = [(np.array([2.5, -.4, 0]), np.array([2.9, .4, 2]))]
        if scenario == "dynamic" and images and t-images[0] >= 3:
            boxes.append((np.array([3.3, -2.0, .6]), np.array([3.55, 2.0, 1.5])))
        clearance = min(float(np.linalg.norm(np.maximum(np.maximum(lo-p, p-hi), 0))) for lo, hi in boxes)
        clearances.append(clearance-.15)
        collisions += int(clearance < .15)
        active = [tr for tr in trajectories if stamp(tr) <= t]
        if active:
            tr = active[-1]
            times = [p.time_from_start.sec + p.time_from_start.nanosec*1e-9 for p in tr.points]
            refs = np.array([vector(p.position) for p in tr.points])
            desired = np.array([np.interp(t-stamp(tr), times, refs[:,j]) for j in range(3)])
            errors.append(float(np.linalg.norm(p-desired)))
    image_control_age = []
    for t, _, _, _ in controls:
        preceding = [im for im in images if im <= t]
        if preceding:
            image_control_age.append((t-preceding[-1])*1000)
    def rate(times):
        return (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
    numerical = {}
    perception_csv = output / "perception.csv"
    if perception_csv.exists():
        with perception_csv.open() as f:
            rows = list(csv.DictReader(f))[3:]
        for key in ("rectify_ms", "disparity_ms", "depth_ms", "cloud_ms", "vo_ms", "estimator_ms", "total_ms", "depth_mae_m", "position_error_m", "valid_fraction"):
            numerical[key] = distribution([float(r[key]) for r in rows])
    matched = [dict(zip(re.findall(r"(\w+)=", line), map(float, re.findall(r"\w+=([\d.]+)", line)))) for line in log.splitlines() if "metrics planning_ms=" in line]
    result = dict(scenario=scenario, terminal=terminal, exit_code=process.returncode,
                  expected_observation_timeout=timed_out and not integrated,
                  mission_timeout=timed_out and integrated, wall_s=wall,
                  images=len(images), frame_hz=rate(images), map_updates=len(maps),
                  control_messages=len(controls), braking_commands=sum(c[3] for c in controls),
                  map_hz=rate([x[0] for x in maps]), estimate_hz=rate([x[0] for x in estimates]),
                  privileged_estimates=sum(e[3] for e in estimates), trajectories=len(trajectories),
                  traveled_m=sum(distances), collision_samples=collisions,
                  minimum_clearance_m=min(clearances) if clearances else None,
                  truth_goal_error_m=float(np.linalg.norm(truth[-1][1]-[4,0,1])) if truth else None,
                  rms_truth_tracking_error_m=math.sqrt(np.mean(np.square(errors))) if errors else None,
                  max_truth_tracking_error_m=max(errors) if errors else None,
                  image_to_control_age_ms=distribution(image_control_age),
                  cpu_seconds=usage.ru_utime+usage.ru_stime-usage_before.ru_utime-usage_before.ru_stime,
                  peak_child_rss_kib=usage.ru_maxrss, numerical=numerical, native_metrics=matched)
    if integrated:
        expected = terminal == "succeeded" if scenario in ("static", "dynamic") else bool(terminal)
        if scenario in ("disconnect", "localization_loss"):
            expected = bool(terminal and terminal.startswith("failed:"))
        if scenario == "dynamic":
            expected = expected and len(trajectories) > 1
        connected = bool(images and maps and estimates) and not collisions and result["privileged_estimates"] == 0
        if scenario == "static":
            connected = connected and bool(trajectories and any(not c[3] for c in controls))
        if scenario in ("disconnect", "localization_loss") and trajectories:
            connected = connected and bool(result["braking_commands"])
        result["passed"] = expected and not timed_out and connected
        if scenario == "disconnect":
            result["fault"] = fault
            result["disconnect_safety"] = disconnect_safety(fault, terminal, data)
            result["passed"] = bool(result["passed"] and process.returncode == 0
                                    and result["disconnect_safety"]["passed"])
    elif scenario == "uvc_unconfigured":
        result["passed"] = process.returncode != 0 and not images and (
            "persistent device path missing or unavailable" in log or
            "cannot open calibration" in log or "calibration" in log and "FATAL" in log)
    else:
        result["passed"] = len(images) > 10 and any(e[2] for e in estimates)
    with (output/"truth.csv").open("w") as f:
        writer = csv.writer(f);writer.writerow(["stamp","x","y","z","vx","vy","vz"])
        writer.writerows([t,*p,*v] for t,p,v in truth)
    (output/"result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: result[k] for k in ("scenario","passed","terminal","images","trajectories","truth_goal_error_m","collision_samples")}), flush=True)
    return result["passed"]


def main():
    import rclpy

    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", nargs="+", default=["static","dynamic","disconnect","localization_loss","synthetic","localization","uvc_unconfigured"])
    parser.add_argument("--output", type=Path, default=Path("results/perception"))
    args = parser.parse_args()
    rclpy.init()
    try:
        results = [run(s, args.output) for s in args.scenarios]
    finally:
        rclpy.shutdown()
    raise SystemExit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
