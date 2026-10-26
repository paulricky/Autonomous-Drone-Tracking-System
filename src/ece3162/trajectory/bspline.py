"""Clamped 3-D B-spline trajectory and sampled feasibility validation."""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.interpolate import BSpline
from numpy.typing import ArrayLike

from ece3162.planning.collision_checker import CollisionChecker


@dataclass(frozen=True, slots=True)
class FeasibilityReport:
    valid: bool
    collision_free: bool
    max_velocity: float
    max_acceleration: float
    velocity_violations: int
    acceleration_violations: int


class BSplineTrajectory:
    def __init__(self, control_points: ArrayLike, duration: float, degree: int = 3) -> None:
        points = np.asarray(control_points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < degree + 1 or not np.all(np.isfinite(points)):
            raise ValueError("control_points must be finite Nx3 with at least degree+1 points")
        if duration <= 0 or degree < 1:
            raise ValueError("duration and degree must be positive")
        self.control_points, self.duration, self.degree = points.copy(), float(duration), int(degree)
        interior_count = len(points) - degree - 1
        interior = np.linspace(0, self.duration, interior_count + 2)[1:-1]
        self.knots = np.concatenate((np.zeros(degree + 1), interior,
                                     np.full(degree + 1, self.duration)))
        self._spline = BSpline(self.knots, self.control_points, degree, extrapolate=False)

    def _times(self, t: ArrayLike) -> np.ndarray:
        values = np.asarray(t, dtype=float)
        if not np.all(np.isfinite(values)) or np.any(values < 0) or np.any(values > self.duration):
            raise ValueError("evaluation time must be within trajectory duration")
        return values

    def position(self, t: ArrayLike) -> np.ndarray: return np.asarray(self._spline(self._times(t)))
    def velocity(self, t: ArrayLike) -> np.ndarray: return np.asarray(self._spline.derivative(1)(self._times(t)))
    def acceleration(self, t: ArrayLike) -> np.ndarray: return np.asarray(self._spline.derivative(2)(self._times(t)))

    def sample(self, dt: float = 0.02) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if dt <= 0: raise ValueError("sample timestep must be positive")
        times = np.linspace(0, self.duration, max(2, int(np.ceil(self.duration/dt))+1))
        return times, self.position(times), self.velocity(times), self.acceleration(times)

    def validate(self, checker: CollisionChecker, max_velocity: float, max_acceleration: float,
                 sample_dt: float = 0.02) -> FeasibilityReport:
        _, p, v, a = self.sample(sample_dt)
        speeds, accelerations = np.linalg.norm(v, axis=1), np.linalg.norm(a, axis=1)
        collision_free = not checker.path_in_collision(p)
        vv, av = int(np.sum(speeds > max_velocity + 1e-9)), int(np.sum(accelerations > max_acceleration + 1e-9))
        return FeasibilityReport(collision_free and vv == 0 and av == 0, collision_free,
                                 float(speeds.max()), float(accelerations.max()), vv, av)
