"""Conservative stop-at-waypoint trapezoidal time parameterization."""

from __future__ import annotations
import math
import numpy as np

from ece3162.common.types import Path, Trajectory, TrajectoryPoint


class TrajectoryGenerator:
    """Generate continuous piecewise-linear motion with bounded acceleration.

    The vehicle stops at path corners. This is conservative but avoids an
    instantaneous velocity-vector change while keeping the geometric path
    collision-safe. Dynamics-aware corner blending is a later extension.
    """

    def __init__(self, max_xy_velocity: float, max_z_velocity: float,
                 max_acceleration: float, sample_rate_hz: float = 20.0) -> None:
        values = (max_xy_velocity, max_z_velocity, max_acceleration, sample_rate_hz)
        if not all(np.isfinite(values)) or min(values) <= 0:
            raise ValueError("trajectory limits and sample rate must be finite and positive")
        self.max_xy_velocity = float(max_xy_velocity)
        self.max_z_velocity = float(max_z_velocity)
        self.max_acceleration = float(max_acceleration)
        self.sample_rate_hz = float(sample_rate_hz)

    def _segment_speed_limit(self, direction: np.ndarray) -> float:
        horizontal = float(np.linalg.norm(direction[:2]))
        limits = []
        if horizontal > 1e-12:
            limits.append(self.max_xy_velocity / horizontal)
        if abs(direction[2]) > 1e-12:
            limits.append(self.max_z_velocity / abs(float(direction[2])))
        return min(limits) if limits else self.max_xy_velocity

    def generate(self, path: Path, yaw: float | None = None) -> Trajectory:
        if not path.valid or len(path.waypoints) == 0:
            raise ValueError("a nonempty valid path is required")
        if len(path.waypoints) == 1:
            zero = np.zeros(3)
            return Trajectory([TrajectoryPoint(path.waypoints[0], zero, zero, 0.0,
                                               yaw=yaw, frame=path.frame)], path.frame)

        points: list[TrajectoryPoint] = []
        elapsed = 0.0
        dt_nominal = 1.0 / self.sample_rate_hz
        zero = np.zeros(3)
        for segment_index, (start, end) in enumerate(zip(path.waypoints[:-1], path.waypoints[1:])):
            delta = end - start
            length = float(np.linalg.norm(delta))
            if length <= 1e-12:
                continue
            direction = delta / length
            speed_limit = self._segment_speed_limit(direction)
            acceleration = self.max_acceleration
            ramp_time = speed_limit / acceleration
            ramp_distance = 0.5 * acceleration * ramp_time**2
            if 2.0 * ramp_distance >= length:
                ramp_time = math.sqrt(length / acceleration)
                cruise_time = 0.0
                peak_speed = acceleration * ramp_time
                ramp_distance = 0.5 * acceleration * ramp_time**2
            else:
                peak_speed = speed_limit
                cruise_time = (length - 2.0 * ramp_distance) / peak_speed
            duration = 2.0 * ramp_time + cruise_time
            sample_count = max(1, int(math.ceil(duration / dt_nominal)))
            local_times = np.linspace(0.0, duration, sample_count + 1)
            for local_index, local_time in enumerate(local_times):
                if segment_index > 0 and local_index == 0:
                    continue
                if local_time < ramp_time:
                    scalar_acceleration = acceleration
                    scalar_velocity = acceleration * local_time
                    distance = 0.5 * acceleration * local_time**2
                elif local_time <= ramp_time + cruise_time:
                    scalar_acceleration = 0.0
                    scalar_velocity = peak_speed
                    distance = ramp_distance + peak_speed * (local_time - ramp_time)
                else:
                    remaining = max(0.0, duration - local_time)
                    scalar_acceleration = -acceleration if remaining > 1e-12 else 0.0
                    scalar_velocity = acceleration * remaining
                    distance = length - 0.5 * acceleration * remaining**2
                position = start + min(length, max(0.0, distance)) * direction
                velocity = scalar_velocity * direction
                accel_vector = scalar_acceleration * direction
                point_yaw = yaw
                if point_yaw is None and np.linalg.norm(direction[:2]) > 1e-12:
                    point_yaw = math.atan2(float(direction[1]), float(direction[0]))
                points.append(TrajectoryPoint(position, velocity, accel_vector,
                                              elapsed + float(local_time), point_yaw,
                                              path.frame))
            elapsed += duration
        if not points:
            raise ValueError("path contains no nonzero-length segments")
        points[-1].position = path.waypoints[-1].copy()
        points[-1].velocity = zero.copy()
        points[-1].acceleration = zero.copy()
        return Trajectory(points, path.frame)

