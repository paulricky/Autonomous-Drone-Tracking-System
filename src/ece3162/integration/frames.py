"""Explicit grid/world and legacy frame boundary utilities."""

from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(frozen=True, slots=True)
class GridFrame2D:
    """A row/column grid embedded in an x/y metric frame.

    Column increases along local +x and row along local +y before ``yaw`` is
    applied. Coordinates refer to cell centers.
    """

    origin_xy: tuple[float, float]
    resolution: float
    yaw: float = 0.0

    def __post_init__(self) -> None:
        values = (*self.origin_xy, self.resolution, self.yaw)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("grid frame values must be finite")
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")

    def grid_to_world(self, row_col: ArrayLike) -> NDArray[np.float64]:
        rc = np.asarray(row_col, dtype=float)
        if rc.shape != (2,) or not np.all(np.isfinite(rc)):
            raise ValueError("row_col must be a finite length-2 coordinate")
        local = np.array([rc[1] + 0.5, rc[0] + 0.5]) * self.resolution
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.asarray(self.origin_xy) + np.array([[c, -s], [s, c]]) @ local

    def world_to_grid(self, xy: ArrayLike) -> tuple[int, int]:
        point = np.asarray(xy, dtype=float)
        if point.shape != (2,) or not np.all(np.isfinite(point)):
            raise ValueError("xy must be a finite length-2 coordinate")
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        local = np.array([[c, s], [-s, c]]) @ (point - np.asarray(self.origin_xy))
        col, row = np.floor(local / self.resolution).astype(int)
        return int(row), int(col)


def planar_path_to_xyz(path: ArrayLike, altitude: float) -> NDArray[np.float64]:
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3) or len(points) == 0:
        raise ValueError("path must be a nonempty finite Nx2 or Nx3 array")
    if not np.all(np.isfinite(points)) or not math.isfinite(altitude):
        raise ValueError("path and altitude must be finite")
    if points.shape[1] == 3:
        return points.copy()
    return np.column_stack((points, np.full(len(points), altitude)))
