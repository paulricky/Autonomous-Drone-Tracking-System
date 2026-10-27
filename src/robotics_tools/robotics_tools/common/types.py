"""Strongly typed messages passed between planning subsystems."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np
from numpy.typing import NDArray

Vector3 = NDArray[np.float64]


class CoordinateFrame(str, Enum):
    """CAMERA is OpenCV; BODY_FLU/LOCAL_ENU are right-handed; PX4 is NED."""
    CAMERA = "camera_opencv"
    BODY_FLU = "body_flu"
    LOCAL_ENU = "local_enu"
    PX4_NED = "px4_ned"


def _vec3(value: Any, name: str) -> Vector3:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite 3-vector; got {value!r}")
    return array.copy()


@dataclass(slots=True)
class DroneState:
    position: Vector3
    velocity: Vector3
    orientation_xyzw: NDArray[np.float64]
    timestamp: float
    frame: CoordinateFrame = CoordinateFrame.LOCAL_ENU

    def __post_init__(self) -> None:
        self.position = _vec3(self.position, "position")
        self.velocity = _vec3(self.velocity, "velocity")
        self.orientation_xyzw = np.asarray(self.orientation_xyzw, dtype=np.float64)
        if self.orientation_xyzw.shape != (4,) or not np.all(np.isfinite(self.orientation_xyzw)):
            raise ValueError("orientation_xyzw must be a finite quaternion")
        if not np.isfinite(self.timestamp):
            raise ValueError("timestamp must be finite")


@dataclass(slots=True)
class Goal:
    position: Vector3
    yaw: float | None = None
    frame: CoordinateFrame = CoordinateFrame.LOCAL_ENU

    def __post_init__(self) -> None:
        self.position = _vec3(self.position, "position")
        if self.yaw is not None and not np.isfinite(self.yaw):
            raise ValueError("yaw must be finite when provided")


@dataclass(slots=True)
class Path:
    waypoints: NDArray[np.float64]
    planner: str
    planning_time: float = 0.0
    valid: bool = True
    frame: CoordinateFrame = CoordinateFrame.LOCAL_ENU

    def __post_init__(self) -> None:
        points = np.asarray(self.waypoints, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError("waypoints must be an Nx3 finite array")
        self.waypoints = points.copy()

    @property
    def total_length(self) -> float:
        if len(self.waypoints) < 2:
            return 0.0
        return float(np.linalg.norm(np.diff(self.waypoints, axis=0), axis=1).sum())


@dataclass(slots=True)
class TrajectoryPoint:
    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    time_from_start: float
    yaw: float | None = None
    frame: CoordinateFrame = CoordinateFrame.LOCAL_ENU

    def __post_init__(self) -> None:
        self.position = _vec3(self.position, "position")
        self.velocity = _vec3(self.velocity, "velocity")
        self.acceleration = _vec3(self.acceleration, "acceleration")
        if self.time_from_start < 0 or not np.isfinite(self.time_from_start):
            raise ValueError("time_from_start must be finite and nonnegative")


@dataclass(slots=True)
class Trajectory:
    points: list[TrajectoryPoint]
    frame: CoordinateFrame = CoordinateFrame.LOCAL_ENU


@dataclass(slots=True)
class MapState:
    occupancy: Any
    timestamp: float
    resolution: float
    frame: CoordinateFrame = CoordinateFrame.LOCAL_ENU


@dataclass(slots=True)
class PlannerResult:
    success: bool
    path: Path | None
    planning_time: float
    expanded_nodes: int = 0
    generated_nodes: int = 0
    iterations: int = 0
    samples: int = 0
    message: str = ""
    diagnostics: dict[str, Any] = field(default_factory=dict)

    @property
    def path_length(self) -> float:
        return self.path.total_length if self.path is not None else float("inf")

