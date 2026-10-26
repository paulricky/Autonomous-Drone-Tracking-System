"""Acceleration-primitive 3-D kinodynamic A* for a double integrator."""

from __future__ import annotations
from dataclasses import dataclass, field
import heapq
import itertools
import math
import time
import numpy as np
from numpy.typing import ArrayLike

from ece3162.mapping.voxel_map import VoxelMap
from .collision_checker import CollisionChecker


@dataclass(frozen=True, slots=True)
class KinodynamicConfig:
    primitive_duration: float = 0.5
    max_velocity: float = 1.5
    max_acceleration: float = 1.0
    velocity_resolution: float = 0.5
    goal_position_tolerance: float = 0.35
    goal_velocity_tolerance: float = 0.55
    max_expansions: int = 100_000
    timeout: float = 10.0
    collision_sample_dt: float = 0.05
    heuristic_weight: float = 1.0


@dataclass(slots=True)
class KinodynamicTrajectory:
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    times: np.ndarray

    @property
    def path_length(self) -> float:
        return float(np.linalg.norm(np.diff(self.positions, axis=0), axis=1).sum())


@dataclass(slots=True)
class KinodynamicResult:
    success: bool
    trajectory: KinodynamicTrajectory | None
    planning_time: float
    expanded_states: int
    message: str


class KinodynamicAStar:
    """Searches quantized (position-cell, velocity) states.

    The heuristic is distance/max_velocity and is a lower bound on remaining
    time when acceleration limits are ignored. Search cost is elapsed time.
    """

    def __init__(self, config: KinodynamicConfig = KinodynamicConfig()) -> None:
        self.config = config
        if min(config.primitive_duration, config.max_velocity, config.max_acceleration,
               config.velocity_resolution, config.timeout, config.collision_sample_dt) <= 0:
            raise ValueError("kinodynamic limits and timing values must be positive")

    def propagate(self, position: ArrayLike, velocity: ArrayLike,
                  acceleration: ArrayLike, duration: float | None = None) -> tuple[np.ndarray, np.ndarray]:
        p, v, a = map(lambda x: np.asarray(x, dtype=float), (position, velocity, acceleration))
        dt = self.config.primitive_duration if duration is None else float(duration)
        if p.shape != (3,) or v.shape != (3,) or a.shape != (3,) or not np.all(np.isfinite([p, v, a])):
            raise ValueError("position, velocity, and acceleration must be finite 3-vectors")
        if dt <= 0:
            raise ValueError("duration must be positive")
        return p + v * dt + 0.5 * a * dt * dt, v + a * dt

    def plan(self, start_position: ArrayLike, goal_position: ArrayLike, voxel_map: VoxelMap,
             initial_velocity: ArrayLike = (0, 0, 0), terminal_velocity: ArrayLike = (0, 0, 0)) -> KinodynamicResult:
        begin = time.perf_counter()
        p0, goal, v0, vg = [np.asarray(x, dtype=float) for x in
                            (start_position, goal_position, initial_velocity, terminal_velocity)]
        if any(x.shape != (3,) or not np.all(np.isfinite(x)) for x in (p0, goal, v0, vg)):
            raise ValueError("states must be finite 3-vectors")
        checker = CollisionChecker(voxel_map, sample_spacing=voxel_map.resolution * 0.2)
        if checker.point_in_collision(p0) or checker.point_in_collision(goal):
            return KinodynamicResult(False, None, time.perf_counter()-begin, 0, "start or goal in collision")
        if np.linalg.norm(v0) > self.config.max_velocity + 1e-9:
            return KinodynamicResult(False, None, time.perf_counter()-begin, 0, "initial velocity exceeds limit")
        # Six axis accelerations plus coasting keep the branching factor bounded.
        # Repeated primitives still produce general 3-D curved motion.
        accelerations = [np.asarray(a, dtype=float) * self.config.max_acceleration
                         for a in itertools.product((-1, 0, 1), repeat=3)
                         if sum(abs(component) for component in a) <= 1]

        def key(p: np.ndarray, v: np.ndarray) -> tuple[int, ...]:
            pi = voxel_map.world_to_grid(p)
            vi = tuple(np.rint(v / self.config.velocity_resolution).astype(int))
            return (*pi, *vi)
        def heuristic(p: np.ndarray) -> float:
            return max(0.0, np.linalg.norm(goal-p)-self.config.goal_position_tolerance) / self.config.max_velocity
        def primitive_clear(p: np.ndarray, v: np.ndarray, a: np.ndarray) -> bool:
            n = max(2, int(math.ceil(self.config.primitive_duration / self.config.collision_sample_dt)))
            return all(not checker.point_in_collision(p + v*t + .5*a*t*t)
                       for t in np.linspace(0, self.config.primitive_duration, n+1))

        try: start_key = key(p0, v0)
        except IndexError:
            return KinodynamicResult(False, None, time.perf_counter()-begin, 0, "start outside map")
        frontier: list[tuple[float, int, tuple[int, ...]]] = [(heuristic(p0), 0, start_key)]
        sequence = itertools.count(1)
        nodes = {start_key: (p0, v0)}
        costs = {start_key: 0.0}
        parents: dict[tuple[int, ...], tuple[tuple[int, ...], np.ndarray]] = {}
        closed: set[tuple[int, ...]] = set()
        found = None
        while frontier and len(closed) < self.config.max_expansions:
            if time.perf_counter() - begin > self.config.timeout:
                return KinodynamicResult(False, None, time.perf_counter()-begin, len(closed), "planning timeout")
            _, _, current = heapq.heappop(frontier)
            if current in closed: continue
            closed.add(current)
            p, v = nodes[current]
            if (np.linalg.norm(p-goal) <= self.config.goal_position_tolerance and
                    np.linalg.norm(v-vg) <= self.config.goal_velocity_tolerance):
                found = current; break
            for a in accelerations:
                pn, vn = self.propagate(p, v, a)
                if np.linalg.norm(vn) > self.config.max_velocity + 1e-9 or not primitive_clear(p, v, a):
                    continue
                try: neighbor = key(pn, vn)
                except IndexError: continue
                candidate = costs[current] + self.config.primitive_duration
                if candidate + 1e-12 >= costs.get(neighbor, math.inf): continue
                nodes[neighbor] = (pn, vn); costs[neighbor] = candidate
                parents[neighbor] = (current, a)
                heapq.heappush(frontier, (candidate + self.config.heuristic_weight*heuristic(pn), next(sequence), neighbor))
        if found is None:
            reason = "expansion limit" if len(closed) >= self.config.max_expansions else "unreachable"
            return KinodynamicResult(False, None, time.perf_counter()-begin, len(closed), reason)
        keys, controls = [found], []
        while keys[-1] != start_key:
            parent, acceleration = parents[keys[-1]]
            controls.append(acceleration); keys.append(parent)
        keys.reverse(); controls.reverse()
        positions = np.asarray([nodes[k][0] for k in keys])
        velocities = np.asarray([nodes[k][1] for k in keys])
        controls_array = np.asarray(controls + [np.zeros(3)])
        times = np.arange(len(keys), dtype=float) * self.config.primitive_duration
        trajectory = KinodynamicTrajectory(positions, velocities, controls_array, times)
        return KinodynamicResult(True, trajectory, time.perf_counter()-begin, len(closed), "trajectory found")
