#!/usr/bin/env python3
"""Run the hardware-free Level 1 algorithm simulation."""

from __future__ import annotations
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]

import numpy as np
from ece3162.mapping.obstacle_inflation import inflate_obstacles
from ece3162.mapping.world_loader import load_world
from ece3162.planning.astar import AStarPlanner
from ece3162.planning.collision_checker import CollisionChecker
from ece3162.planning.path_utils import shortcut_path
from ece3162.utils.config import load_yaml
from ece3162.visualization.planning_view import show_planning_result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", choices=("astar", "rrt", "rrt_star"), default="astar")
    parser.add_argument("--world", default="single_obstacle",
                        help="world directory name under simulation/worlds")
    parser.add_argument("--no-show", action="store_true", help="do not open the matplotlib window")
    parser.add_argument("--save", help="optional output image path")
    parser.add_argument("--hide-explored", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.planner != "astar":
        print(f"Planner '{args.planner}' is reserved for the next implementation phase.", file=sys.stderr)
        return 2
    planning = load_yaml(ROOT / "config/planning.yaml")
    mapping = load_yaml(ROOT / "config/mapping.yaml")
    world_file = ROOT / "simulation/worlds" / args.world / "world.yaml"
    world = load_world(world_file)
    inflated = inflate_obstacles(world.voxel_map, float(mapping["drone_radius"]),
                                 float(mapping["safety_margin"]))
    planner = AStarPlanner(connectivity=int(planning["connectivity"]),
                           heuristic_weight=float(planning["heuristic_weight"]),
                           allow_unknown=bool(planning["allow_unknown"]))
    result = planner.plan(world.start, world.goal, inflated)
    print(f"World:          {world.name}")
    print(f"Planner:        astar ({planner.connectivity}-connected)")
    print(f"Planning time:  {result.planning_time * 1000.0:.2f} ms")
    print(f"Expanded nodes: {result.expanded_nodes}")
    print(f"Generated nodes:{result.generated_nodes:>7}")
    print(f"Success:        {result.success}")
    if not result.success or result.path is None:
        print(result.message)
        return 1
    checker = CollisionChecker(inflated, unknown_is_occupied=not planner.allow_unknown)
    smooth = shortcut_path(result.path, checker)
    print(f"Raw path:       {result.path.total_length:.3f} m ({len(result.path.waypoints)} points)")
    print(f"Smoothed path:  {smooth.total_length:.3f} m ({len(smooth.waypoints)} points)")
    explored = None
    if not args.hide_explored:
        explored = np.asarray([inflated.grid_to_world(index) for index in planner.explored_indices])
    show_planning_result(world.voxel_map, inflated, world.start, world.goal,
                         result.path, smooth, explored, show=not args.no_show,
                         save_path=args.save,
                         title=f"{world.name}: A* in inflated configuration space")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
