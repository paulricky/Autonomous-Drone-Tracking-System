"""Safe ROS-free boundary for the recovered Duckiedrone position interface."""

from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import math
import numpy as np
from numpy.typing import ArrayLike


LEGACY_ABSOLUTE_POSE_TOPIC = "/pidrone/desired/pose/absolute"
LEGACY_STATE_TOPIC = "/pidrone/state"


@dataclass(frozen=True, slots=True)
class PositionCommand:
    x: float
    y: float
    z: float
    timestamp: float


class CommandPublisher(Protocol):
    def publish(self, command: PositionCommand) -> None: ...


class MemoryPublisher:
    def __init__(self) -> None:
        self.commands: list[PositionCommand] = []
    def publish(self, command: PositionCommand) -> None:
        self.commands.append(command)


class DuckiedroneBridge:
    """Gates commands; it never imports ROS and never arms the aircraft."""

    def __init__(self, publisher: CommandPublisher, *, hardware_enabled: bool = False,
                 max_command_rate_hz: float = 20.0) -> None:
        if max_command_rate_hz <= 0 or not math.isfinite(max_command_rate_hz):
            raise ValueError("max_command_rate_hz must be finite and positive")
        self.publisher = publisher
        self.hardware_enabled = bool(hardware_enabled)
        self.active = False
        self.period = 1.0 / max_command_rate_hz
        self._last_publish = -math.inf

    def activate(self, *, preflight_ok: bool, state_fresh: bool) -> None:
        if not self.hardware_enabled:
            raise PermissionError("hardware publication is disabled")
        if not preflight_ok or not state_fresh:
            raise RuntimeError("preflight and fresh state are required")
        self.active = True

    def deactivate(self) -> None:
        """Stop navigation publication; this is not hold, land, or disarm."""
        self.active = False

    def publish_waypoint(self, waypoint: ArrayLike, now: float, *, state_fresh: bool) -> bool:
        point = np.asarray(waypoint, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("waypoint must be a finite 3-vector")
        if not self.active or not state_fresh:
            self.active = False if not state_fresh else self.active
            return False
        if now - self._last_publish + 1e-12 < self.period:
            return False
        self.publisher.publish(PositionCommand(*map(float, point), float(now)))
        self._last_publish = now
        return True
