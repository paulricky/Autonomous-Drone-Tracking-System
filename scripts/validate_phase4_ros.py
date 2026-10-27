#!/usr/bin/env python3
"""ROS-only observer: never publishes map, localization or control inputs.

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
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from flight_interfaces.msg import VehicleState, TimedTrajectory, ControlCommand


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


def run(scenario, root):
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
    elif scenario == "disconnect":
        command += ["disconnect_after_frames:=30"]
    elif scenario == "localization_loss":
        command += ["low_texture_after_frames:=30"]
    node = rclpy.create_node("phase4_evaluator")
    status, truth, estimates, controls, images, maps, trajectories = [], [], [], [], [], [], []
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
    ]
    usage_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    begun = time.monotonic()
    with (output / "launch.log").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        limit = 55 if integrated else 7
        try:
            while process.poll() is None and time.monotonic() - begun < limit:
                rclpy.spin_once(node, timeout_sec=.1)
            timed_out = process.poll() is None
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
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
    elif scenario == "uvc_unconfigured":
        result["passed"] = "cannot open calibration" in log or "calibration" in log and "FATAL" in log
    else:
        result["passed"] = len(images) > 10 and any(e[2] for e in estimates)
    with (output/"truth.csv").open("w") as f:
        writer = csv.writer(f);writer.writerow(["stamp","x","y","z","vx","vy","vz"])
        writer.writerows([t,*p,*v] for t,p,v in truth)
    (output/"result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: result[k] for k in ("scenario","passed","terminal","images","trajectories","truth_goal_error_m","collision_samples")}), flush=True)
    return result["passed"]


def main():
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
