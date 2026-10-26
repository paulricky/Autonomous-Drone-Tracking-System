"""ROS-independent feed-forward PD tracking for translational simulation."""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import numpy as np

from ece3162.trajectory.bspline import BSplineTrajectory


class TrackingStatus(str, Enum):
    ACTIVE="active"; COMPLETE="complete"; FAILED="failed"; CANCELLED="cancelled"


@dataclass(frozen=True, slots=True)
class TrackerConfig:
    kp: float = 4.0
    kd: float = 2.8
    max_acceleration: float = 2.0
    max_tracking_error: float = 2.0
    position_tolerance: float = 0.15
    velocity_tolerance: float = 0.15
    localization_timeout: float = 0.25
    completion_timeout: float = 3.0


@dataclass(slots=True)
class TrackingCommand:
    acceleration: np.ndarray
    desired_position: np.ndarray
    desired_velocity: np.ndarray
    error: float
    saturated: bool


class TrajectoryTracker:
    def __init__(self, config: TrackerConfig = TrackerConfig()) -> None:
        self.config = config; self.trajectory: BSplineTrajectory | None = None
        self.started_at = 0.0; self.status = TrackingStatus.CANCELLED; self.failure_reason = None

    def set_trajectory(self, trajectory: BSplineTrajectory, now: float) -> None:
        self.trajectory, self.started_at, self.status, self.failure_reason = trajectory, now, TrackingStatus.ACTIVE, None

    def cancel(self, reason: str="operator cancellation") -> None:
        self.status, self.failure_reason = TrackingStatus.CANCELLED, reason

    def update(self, now: float, position: np.ndarray, velocity: np.ndarray,
               state_timestamp: float, state_valid: bool=True) -> TrackingCommand | None:
        if self.status != TrackingStatus.ACTIVE or self.trajectory is None: return None
        p, v = np.asarray(position, float), np.asarray(velocity, float)
        if (not state_valid or p.shape != (3,) or v.shape != (3,) or
                not np.all(np.isfinite([p, v])) or now-state_timestamp > self.config.localization_timeout):
            self.status, self.failure_reason = TrackingStatus.FAILED, "invalid or stale localization"; return None
        elapsed = max(0.0, now-self.started_at); t = min(elapsed, self.trajectory.duration)
        pd = self.trajectory.position(t)
        if elapsed >= self.trajectory.duration:
            # Terminal velocity is zero: hold the endpoint while the simulated
            # vehicle settles instead of extrapolating the spline derivative.
            vd, ad = np.zeros(3), np.zeros(3)
        else:
            vd = self.trajectory.velocity(t); ad = self.trajectory.acceleration(t)
        error = float(np.linalg.norm(pd-p))
        if error > self.config.max_tracking_error:
            self.status, self.failure_reason = TrackingStatus.FAILED, "excessive tracking error"; return None
        command = ad + self.config.kp*(pd-p) + self.config.kd*(vd-v)
        norm = float(np.linalg.norm(command)); saturated = norm > self.config.max_acceleration
        if saturated: command *= self.config.max_acceleration/norm
        if elapsed >= self.trajectory.duration:
            at_goal = error <= self.config.position_tolerance and np.linalg.norm(v) <= self.config.velocity_tolerance
            if at_goal: self.status = TrackingStatus.COMPLETE
            elif elapsed > self.trajectory.duration + self.config.completion_timeout:
                self.status, self.failure_reason = TrackingStatus.FAILED, "goal settling timeout"
        return TrackingCommand(command, np.asarray(pd), np.asarray(vd), error, saturated)
