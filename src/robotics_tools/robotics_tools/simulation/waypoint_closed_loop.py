"""Simplified deterministic closed-loop position/velocity simulation."""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from robotics_tools.integration.waypoint_manager import MissionState, WaypointManager


@dataclass(frozen=True, slots=True)
class PointMassConfig:
    dt: float = 0.05
    position_gain: float = 1.8
    max_speed: float = 1.0
    max_acceleration: float = 1.5
    max_time: float = 120.0


@dataclass(slots=True)
class SimulationResult:
    times: np.ndarray
    positions: np.ndarray
    desired: np.ndarray
    errors: np.ndarray
    status: MissionState
    failure_reason: str | None


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0
    def __call__(self) -> float:
        return self.now


def run_closed_loop(manager: WaypointManager, initial_position: np.ndarray,
                    config: PointMassConfig = PointMassConfig()) -> SimulationResult:
    """Track manager setpoints with bounded velocity and acceleration."""
    if config.dt <= 0 or config.max_speed <= 0 or config.max_acceleration <= 0:
        raise ValueError("simulation timestep and limits must be positive")
    clock = manager.clock
    if not isinstance(clock, ManualClock):
        raise TypeError("closed-loop simulation requires ManualClock")
    position = np.asarray(initial_position, dtype=float).copy()
    velocity = np.zeros(3)
    manager.start()
    times, positions, desireds, errors = [], [], [], []
    while clock.now <= config.max_time and manager.state == MissionState.ACTIVE:
        manager.update(position, clock.now)
        target = manager.desired
        if target is None:
            break
        error = target - position
        requested = config.position_gain * error
        speed = np.linalg.norm(requested)
        if speed > config.max_speed:
            requested *= config.max_speed / speed
        delta_v = requested - velocity
        max_delta = config.max_acceleration * config.dt
        if np.linalg.norm(delta_v) > max_delta:
            delta_v *= max_delta / np.linalg.norm(delta_v)
        velocity += delta_v
        position += velocity * config.dt
        times.append(clock.now)
        positions.append(position.copy())
        desireds.append(target.copy())
        errors.append(float(np.linalg.norm(error)))
        clock.now += config.dt
    if manager.state == MissionState.ACTIVE:
        manager._fail("simulation time limit")
    return SimulationResult(np.asarray(times), np.asarray(positions), np.asarray(desireds),
                            np.asarray(errors), manager.state, manager.failure_reason)
