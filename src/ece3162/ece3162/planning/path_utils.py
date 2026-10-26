"""Path post-processing utilities."""

from __future__ import annotations
import numpy as np
from ece3162.common.types import Path
from .collision_checker import CollisionChecker


def shortcut_path(path: Path, checker: CollisionChecker) -> Path:
    """Greedily connect each waypoint to its farthest visible successor."""
    points = path.waypoints
    if len(points) <= 2:
        return Path(points.copy(), path.planner, path.planning_time, path.valid, path.frame)
    shortened = [points[0]]
    current = 0
    while current < len(points) - 1:
        candidate = len(points) - 1
        while candidate > current + 1 and checker.segment_in_collision(points[current], points[candidate]):
            candidate -= 1
        shortened.append(points[candidate])
        current = candidate
    result = Path(np.asarray(shortened), f"{path.planner}+shortcut", path.planning_time,
                  True, path.frame)
    if checker.path_in_collision(result):
        raise RuntimeError("shortcut algorithm produced an invalid path")
    return result


def minimum_clearance(path: Path, occupied_centers: np.ndarray) -> float:
    if len(occupied_centers) == 0:
        return float("inf")
    return float(min(np.linalg.norm(occupied_centers - point, axis=1).min()
                     for point in path.waypoints))

