#!/usr/bin/env python3
"""Plan and execute a closed-loop, ROS-free Duckiedrone simulation."""

from __future__ import annotations
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

import matplotlib.pyplot as plt
import numpy as np
from ece3162.integration.waypoint_manager import WaypointConfig, WaypointManager
from ece3162.mapping.obstacle_inflation import inflate_obstacles
from ece3162.mapping.world_loader import load_world
from ece3162.planning.astar import AStarPlanner
from ece3162.planning.collision_checker import CollisionChecker
from ece3162.planning.path_utils import shortcut_path
from ece3162.simulation.waypoint_closed_loop import ManualClock, PointMassConfig, run_closed_loop
from ece3162.utils.config import load_yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("baseline", "obstacle"), default="baseline")
    parser.add_argument("--output-dir", default="benchmark_results/duckiedrone")
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def point_box_clearance(point: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(lower - point, point - upper), 0.0)))


def main() -> int:
    args = parse_args()
    cfg = load_yaml(ROOT / "config/duckiedrone_simulation.yaml")
    if cfg["hardware_publishing_enabled"]:
        raise RuntimeError("simulation configuration must not enable hardware")
    if args.scenario == "baseline":
        world = load_world(ROOT / "simulation/worlds/duckiedrone_baseline/world.yaml")
    else:
        world = load_world(ROOT / "simulation/worlds/multi_obstacle/world.yaml")
    inflated = inflate_obstacles(world.voxel_map, cfg["drone_radius"], cfg["safety_margin"])
    result = AStarPlanner(connectivity=26).plan(world.start, world.goal, inflated)
    if not result.success or result.path is None:
        print(json.dumps({"scenario": args.scenario, "status": "planning_failed", "reason": result.message}))
        return 1
    path = shortcut_path(result.path, CollisionChecker(inflated))
    clock = ManualClock()
    manager = WaypointManager(WaypointConfig(
        target_altitude=cfg["target_altitude"], horizontal_tolerance=cfg["horizontal_tolerance"],
        vertical_tolerance=cfg["vertical_tolerance"], dwell_time=cfg["dwell_time"],
        state_timeout=cfg["state_timeout"], waypoint_timeout=cfg["waypoint_timeout"],
        mission_timeout=cfg["mission_timeout"]), clock)
    manager.load(path.waypoints)
    sim = run_closed_loop(manager, world.start, PointMassConfig(
        dt=cfg["dt"], position_gain=cfg["position_gain"], max_speed=cfg["max_speed"],
        max_acceleration=cfg["max_acceleration"], max_time=cfg["mission_timeout"]))
    actual_length = float(np.linalg.norm(np.diff(sim.positions, axis=0), axis=1).sum()) if len(sim.positions) > 1 else 0.0
    goal_error = float(np.linalg.norm(sim.positions[-1] - world.goal))
    clearances = [point_box_clearance(p, lo, hi) - cfg["drone_radius"]
                  for p in sim.positions for lo, hi in world.obstacle_boxes]
    minimum_clearance = min(clearances) if clearances else None
    collisions = sum(any(point_box_clearance(p, lo, hi) <= cfg["drone_radius"]
                         for lo, hi in world.obstacle_boxes) for p in sim.positions)
    metrics = {"scenario": args.scenario, "planner": "astar", "status": sim.status.value,
               "failure_reason": sim.failure_reason, "goal_reached": sim.status.value == "complete",
               "planned_path_length_m": path.total_length, "actual_path_length_m": actual_length,
               "final_position_error_m": goal_error, "rms_tracking_error_m": float(np.sqrt(np.mean(sim.errors ** 2))),
               "maximum_tracking_error_m": float(np.max(sim.errors)),
               "mission_completion_time_s": float(sim.times[-1]),
               "minimum_obstacle_clearance_m": minimum_clearance, "collision_count": int(collisions)}
    out = ROOT / args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{args.scenario}_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    fig, (ax, error_ax) = plt.subplots(1, 2, figsize=(14, 6))
    for lower, upper in world.obstacle_boxes:
        ax.add_patch(plt.Rectangle(lower[:2], *(upper[:2] - lower[:2]), color="0.5", alpha=0.7))
    ax.plot(path.waypoints[:, 0], path.waypoints[:, 1], "o--", label="A* waypoints")
    ax.plot(sim.positions[:, 0], sim.positions[:, 1], "-", linewidth=2, label="executed")
    ax.scatter(*world.start[:2], marker="o", s=100, label="start")
    ax.scatter(*world.goal[:2], marker="*", s=180, label="goal")
    ax.set(xlabel="x (m)", ylabel="y (m)", title=f"{args.scenario}: altitude {cfg['target_altitude']:.1f} m")
    ax.axis("equal"); ax.grid(True); ax.legend()
    error_ax.plot(sim.times, sim.errors)
    error_ax.set(xlabel="time (s)", ylabel="setpoint error (m)", title="Tracking error")
    error_ax.grid(True)
    fig.tight_layout(); fig.savefig(out / f"{args.scenario}_trajectory.png", dpi=180)
    if args.show: plt.show()
    plt.close(fig)
    print(json.dumps(metrics, indent=2))
    return 0 if metrics["goal_reached"] and collisions == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
