"""Continuous interpolation of a time-parameterized trajectory."""

from __future__ import annotations
from dataclasses import dataclass
import bisect
import math
import numpy as np

from common.types import Trajectory


@dataclass(slots=True)
class KinematicSample:
    position: np.ndarray
    velocity: np.ndarray
    yaw: float
    complete: bool


class KinematicTrajectoryFollower:
    def __init__(self, trajectory: Trajectory, speed_scale: float = 1.0) -> None:
        if not trajectory.points:
            raise ValueError("trajectory cannot be empty")
        if not np.isfinite(speed_scale) or speed_scale <= 0:
            raise ValueError("speed_scale must be finite and positive")
        self.trajectory = trajectory
        self.speed_scale = float(speed_scale)
        self._times = [point.time_from_start for point in trajectory.points]

    @property
    def duration(self) -> float:
        return self._times[-1] / self.speed_scale

    def sample(self, elapsed: float) -> KinematicSample:
        trajectory_time = max(0.0, float(elapsed)) * self.speed_scale
        if trajectory_time >= self._times[-1]:
            final = self.trajectory.points[-1]
            return KinematicSample(final.position.copy(), np.zeros(3),
                                   float(final.yaw or 0.0), True)
        right = bisect.bisect_right(self._times, trajectory_time)
        left = max(0, right - 1)
        right = min(right, len(self._times) - 1)
        first = self.trajectory.points[left]
        second = self.trajectory.points[right]
        interval = second.time_from_start - first.time_from_start
        ratio = 0.0 if interval <= 1e-12 else (
            trajectory_time - first.time_from_start) / interval
        position = first.position + ratio * (second.position - first.position)
        velocity = first.velocity + ratio * (second.velocity - first.velocity)
        yaw_a = float(first.yaw or 0.0)
        yaw_b = float(second.yaw or yaw_a)
        yaw_delta = math.atan2(math.sin(yaw_b - yaw_a), math.cos(yaw_b - yaw_a))
        return KinematicSample(position, velocity, yaw_a + ratio * yaw_delta, False)

