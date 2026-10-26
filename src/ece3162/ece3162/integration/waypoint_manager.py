"""Deterministic, ROS-free waypoint mission state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Callable
import numpy as np
from numpy.typing import ArrayLike, NDArray

from .frames import planar_path_to_xyz


class MissionState(str, Enum):
    IDLE = "idle"
    ACTIVE = "active"
    PAUSED = "paused"
    COMPLETE = "complete"
    CANCELLED = "cancelled"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class WaypointConfig:
    target_altitude: float = 1.0
    horizontal_tolerance: float = 0.12
    vertical_tolerance: float = 0.08
    dwell_time: float = 0.15
    state_timeout: float = 0.5
    waypoint_timeout: float = 30.0
    mission_timeout: float = 120.0
    duplicate_tolerance: float = 1e-6

    def __post_init__(self) -> None:
        vals = (self.target_altitude, self.horizontal_tolerance,
                self.vertical_tolerance, self.dwell_time, self.state_timeout,
                self.waypoint_timeout, self.mission_timeout, self.duplicate_tolerance)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError("waypoint configuration must be finite")
        if min(self.horizontal_tolerance, self.vertical_tolerance,
               self.state_timeout, self.waypoint_timeout, self.mission_timeout) <= 0:
            raise ValueError("tolerances and timeouts must be positive")
        if self.dwell_time < 0 or self.duplicate_tolerance < 0:
            raise ValueError("dwell and duplicate tolerance cannot be negative")


class WaypointManager:
    def __init__(self, config: WaypointConfig = WaypointConfig(),
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.config, self.clock = config, clock
        self.waypoints = np.empty((0, 3))
        self.state = MissionState.IDLE
        self.index = 0
        self.failure_reason: str | None = None
        self._mission_started = self._waypoint_started = 0.0
        self._inside_since: float | None = None
        self._pause_started: float | None = None

    def load(self, path: ArrayLike) -> None:
        points = planar_path_to_xyz(path, self.config.target_altitude)
        retained = [points[0]]
        for point in points[1:]:
            if np.linalg.norm(point - retained[-1]) > self.config.duplicate_tolerance:
                retained.append(point)
        self.waypoints = np.asarray(retained)
        self.state, self.index, self.failure_reason = MissionState.IDLE, 0, None

    @property
    def desired(self) -> NDArray[np.float64] | None:
        if self.state not in (MissionState.ACTIVE, MissionState.PAUSED) or not len(self.waypoints):
            return None
        return self.waypoints[self.index].copy()

    def start(self) -> None:
        if not len(self.waypoints):
            raise RuntimeError("load a valid path before starting")
        now = self.clock()
        self.state = MissionState.ACTIVE
        self._mission_started = self._waypoint_started = now

    def update(self, position: ArrayLike | None, state_timestamp: float | None) -> MissionState:
        if self.state != MissionState.ACTIVE:
            return self.state
        now = self.clock()
        if position is None or state_timestamp is None:
            return self._fail("missing state feedback")
        point = np.asarray(position, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)) or not math.isfinite(state_timestamp):
            return self._fail("invalid state feedback")
        if state_timestamp > now + 1e-9 or now - state_timestamp > self.config.state_timeout:
            return self._fail("stale state feedback")
        if now - self._mission_started > self.config.mission_timeout:
            return self._fail("mission timeout")
        if now - self._waypoint_started > self.config.waypoint_timeout:
            return self._fail("waypoint timeout")
        error = self.waypoints[self.index] - point
        inside = np.linalg.norm(error[:2]) <= self.config.horizontal_tolerance and abs(error[2]) <= self.config.vertical_tolerance
        if not inside:
            self._inside_since = None
            return self.state
        if self._inside_since is None:
            self._inside_since = now
        if now - self._inside_since + 1e-12 < self.config.dwell_time:
            return self.state
        if self.index == len(self.waypoints) - 1:
            self.state = MissionState.COMPLETE
        else:
            self.index += 1
            self._waypoint_started, self._inside_since = now, None
        return self.state

    def pause(self) -> None:
        if self.state == MissionState.ACTIVE:
            self.state, self._pause_started = MissionState.PAUSED, self.clock()

    def resume(self) -> None:
        if self.state == MissionState.PAUSED:
            now = self.clock()
            pause_started = now if self._pause_started is None else self._pause_started
            delta = now - pause_started
            self._mission_started += delta
            self._waypoint_started += delta
            if self._inside_since is not None:
                self._inside_since += delta
            self.state, self._pause_started = MissionState.ACTIVE, None

    def cancel(self, reason: str = "cancelled by operator") -> None:
        if self.state not in (MissionState.COMPLETE, MissionState.FAILED):
            self.state, self.failure_reason = MissionState.CANCELLED, reason

    def _fail(self, reason: str) -> MissionState:
        self.state, self.failure_reason = MissionState.FAILED, reason
        return self.state
