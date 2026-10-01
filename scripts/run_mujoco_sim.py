#!/usr/bin/env python3
"""Plan an A* route and execute it continuously with the MuJoCo F450."""

from __future__ import annotations
import argparse
import os
import platform
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

from mapping.obstacle_inflation import inflate_obstacles
from mapping.world_loader import load_world
from planning.astar import AStarPlanner
from planning.collision_checker import CollisionChecker
from planning.path_utils import shortcut_path
from simulation.mujoco_runner import MujocoKinematicRunner
from trajectory.time_parameterization import TrajectoryGenerator
from utils.config import load_yaml


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="single_obstacle")
    parser.add_argument("--planner", choices=("astar", "rrt", "rrt_star"), default="astar")
    parser.add_argument("--show-path", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--speed", type=float, default=1.0, help="positive playback speed scale")
    parser.add_argument("--headless", action="store_true", help="validate without opening a viewer")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    if platform.system() == "Darwin" and not args.headless and "MJPYTHON_BIN" not in os.environ:
        launcher = Path(sys.executable).with_name("mjpython")
        if not launcher.is_file():
            print("MuJoCo viewer on macOS requires mjpython, but it was not found next to "
                  f"the active interpreter: {launcher}", file=sys.stderr)
            return 2
        os.execv(str(launcher), [str(launcher), *sys.argv])
    if args.planner != "astar":
        print(f"Planner not implemented yet: {args.planner}", file=sys.stderr)
        return 2
    if args.speed <= 0:
        print("--speed must be positive", file=sys.stderr)
        return 2
    yaml_path = ROOT / "simulation/worlds" / args.world / "world.yaml"
    xml_path = ROOT / "simulation/worlds" / f"{args.world}.xml"
    if not yaml_path.is_file() or not xml_path.is_file():
        print(f"MuJoCo world not available: {args.world}", file=sys.stderr)
        return 2
    planning = load_yaml(ROOT / "config/planning.yaml")
    mapping = load_yaml(ROOT / "config/mapping.yaml")
    trajectory_config = load_yaml(ROOT / "config/trajectory.yaml")
    world = load_world(yaml_path)
    inflated = inflate_obstacles(world.voxel_map, mapping["drone_radius"], mapping["safety_margin"])
    planner = AStarPlanner(connectivity=planning["connectivity"],
                           heuristic_weight=planning["heuristic_weight"],
                           allow_unknown=planning["allow_unknown"])
    result = planner.plan(world.start, world.goal, inflated)
    if not result.success or result.path is None:
        print(result.message or "NO COLLISION-FREE PATH FOUND", file=sys.stderr)
        return 1
    checker = CollisionChecker(inflated, unknown_is_occupied=not planner.allow_unknown)
    route = shortcut_path(result.path, checker)
    generator = TrajectoryGenerator(trajectory_config["max_xy_velocity"],
                                    trajectory_config["max_z_velocity"],
                                    trajectory_config["max_acceleration"],
                                    trajectory_config["setpoint_rate_hz"])
    trajectory = generator.generate(route)
    runner = MujocoKinematicRunner(xml_path, trajectory, route, args.speed, args.show_path)
    print("Simulation mode: MUJOCO KINEMATIC (ground-truth map)")
    print(f"Planner: {args.planner} | planning: {result.planning_time * 1000:.1f} ms")
    print(f"Path: {result.path.total_length:.3f} m raw, {route.total_length:.3f} m smoothed")
    print(f"Trajectory: {trajectory.points[-1].time_from_start:.2f} s, {len(trajectory.points)} samples")
    final = runner.run_headless() if args.headless else runner.run_interactive()
    error = float(np.linalg.norm(final - world.goal))
    print(f"Final position: {np.round(final, 3).tolist()} | goal error: {error:.6f} m")
    return 0 if error <= 1e-6 else 1


if __name__ == "__main__":
    raise SystemExit(main())
