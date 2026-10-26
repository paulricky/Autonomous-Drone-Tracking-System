"""True 3-D A* over a voxel configuration space."""

from __future__ import annotations
import heapq
import itertools
import time
import numpy as np
from numpy.typing import ArrayLike

from ece3162.common.types import Path, PlannerResult
from ece3162.mapping.voxel_map import VoxelMap
from .collision_checker import CollisionChecker
from .planner import MotionPlanner


GridIndex = tuple[int, int, int]


class AStarPlanner(MotionPlanner):
    def __init__(self, *, connectivity: int = 26, heuristic_weight: float = 1.0,
                 allow_unknown: bool = False) -> None:
        if connectivity not in (6, 18, 26):
            raise ValueError("connectivity must be 6, 18, or 26")
        if not 0.0 <= heuristic_weight <= 1.0:
            raise ValueError("heuristic_weight must be in [0, 1] for admissible A*")
        self.connectivity = connectivity
        self.heuristic_weight = float(heuristic_weight)
        self.allow_unknown = bool(allow_unknown)
        self.explored_indices: list[GridIndex] = []

    def _offsets(self) -> list[GridIndex]:
        offsets = []
        for delta in itertools.product((-1, 0, 1), repeat=3):
            nonzero = sum(value != 0 for value in delta)
            if nonzero == 0:
                continue
            if self.connectivity == 6 and nonzero != 1:
                continue
            if self.connectivity == 18 and nonzero == 3:
                continue
            offsets.append(delta)
        return offsets

    def plan(self, start: ArrayLike, goal: ArrayLike, voxel_map: VoxelMap) -> PlannerResult:
        begin = time.perf_counter()
        self.reset()
        checker = CollisionChecker(voxel_map, unknown_is_occupied=not self.allow_unknown)
        start_array = np.asarray(start, dtype=float)
        goal_array = np.asarray(goal, dtype=float)
        try:
            start_index = voxel_map.world_to_grid(start_array)
            goal_index = voxel_map.world_to_grid(goal_array)
        except IndexError as exc:
            return PlannerResult(False, None, time.perf_counter() - begin, message=str(exc))
        if checker.point_in_collision(start_array):
            return PlannerResult(False, None, time.perf_counter() - begin,
                                 message="start is in collision or unknown space")
        if checker.point_in_collision(goal_array):
            return PlannerResult(False, None, time.perf_counter() - begin,
                                 message="goal is in collision or unknown space")

        frontier: list[tuple[float, int, GridIndex]] = []
        sequence = itertools.count()
        heapq.heappush(frontier, (0.0, next(sequence), start_index))
        came_from: dict[GridIndex, GridIndex] = {}
        cost = {start_index: 0.0}
        generated = 1
        offsets = self._offsets()
        closed: set[GridIndex] = set()

        while frontier:
            _, _, current = heapq.heappop(frontier)
            if current in closed:
                continue
            closed.add(current)
            self.explored_indices.append(current)
            if current == goal_index:
                indices = [current]
                while indices[-1] != start_index:
                    indices.append(came_from[indices[-1]])
                indices.reverse()
                points = np.asarray([voxel_map.grid_to_world(index) for index in indices])
                points[0] = start_array
                points[-1] = goal_array
                path = Path(points, "astar", time.perf_counter() - begin)
                if checker.path_in_collision(path):
                    return PlannerResult(False, None, time.perf_counter() - begin,
                                         len(self.explored_indices), generated,
                                         message="reconstructed path failed continuous collision check")
                return PlannerResult(True, path, time.perf_counter() - begin,
                                     len(self.explored_indices), generated,
                                     message="path found",
                                     diagnostics={"explored_indices": self.explored_indices.copy()})

            current_world = voxel_map.grid_to_world(current)
            for offset in offsets:
                neighbor = tuple(current[axis] + offset[axis] for axis in range(3))
                if not voxel_map.contains_index(neighbor):
                    continue
                neighbor_world = voxel_map.grid_to_world(neighbor)
                if checker.segment_in_collision(current_world, neighbor_world):
                    continue
                edge_cost = float(np.linalg.norm(np.asarray(offset))) * voxel_map.resolution
                candidate_cost = cost[current] + edge_cost
                if candidate_cost >= cost.get(neighbor, float("inf")):
                    continue
                first_seen = neighbor not in cost
                cost[neighbor] = candidate_cost
                came_from[neighbor] = current
                heuristic = np.linalg.norm(np.asarray(neighbor) - np.asarray(goal_index)) * voxel_map.resolution
                heapq.heappush(frontier, (candidate_cost + self.heuristic_weight * heuristic,
                                         next(sequence), neighbor))
                generated += int(first_seen)

        return PlannerResult(False, None, time.perf_counter() - begin,
                             len(self.explored_indices), generated,
                             message="NO COLLISION-FREE PATH FOUND")

    def is_path_valid(self, path: Path, voxel_map: VoxelMap) -> bool:
        return not CollisionChecker(voxel_map,
                                    unknown_is_occupied=not self.allow_unknown).path_in_collision(path)

    def reset(self) -> None:
        self.explored_indices = []
