"""Point, segment, path, and sampled trajectory collision checks."""

from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike

from common.types import Path, Trajectory
from mapping.voxel_map import Occupancy, VoxelMap


class CollisionChecker:
    def __init__(self, voxel_map: VoxelMap, *, unknown_is_occupied: bool = True,
                 sample_spacing: float | None = None) -> None:
        self.map = voxel_map
        self.unknown_is_occupied = bool(unknown_is_occupied)
        self.sample_spacing = sample_spacing or voxel_map.resolution * 0.25
        if self.sample_spacing <= 0:
            raise ValueError("sample spacing must be positive")

    def point_in_collision(self, point: ArrayLike) -> bool:
        try:
            value = Occupancy(int(self.map.data[self.map.world_to_grid(point)]))
        except IndexError:
            return True
        return value == Occupancy.OCCUPIED or (
            self.unknown_is_occupied and value == Occupancy.UNKNOWN)

    def segment_in_collision(self, start: ArrayLike, end: ArrayLike) -> bool:
        start_array = np.asarray(start, dtype=float)
        end_array = np.asarray(end, dtype=float)
        distance = float(np.linalg.norm(end_array - start_array))
        samples = max(1, int(np.ceil(distance / self.sample_spacing)))
        for fraction in np.linspace(0.0, 1.0, samples + 1):
            if self.point_in_collision(start_array + fraction * (end_array - start_array)):
                return True
        return False

    def path_in_collision(self, path: Path | ArrayLike) -> bool:
        points = path.waypoints if isinstance(path, Path) else np.asarray(path, dtype=float)
        if len(points) == 0:
            return True
        if len(points) == 1:
            return self.point_in_collision(points[0])
        return any(self.segment_in_collision(a, b) for a, b in zip(points[:-1], points[1:]))

    def trajectory_in_collision(self, trajectory: Trajectory) -> bool:
        return self.path_in_collision(np.asarray([point.position for point in trajectory.points]))

