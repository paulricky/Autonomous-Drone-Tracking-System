"""Dense local 3-D occupancy map with explicit unknown space."""

from __future__ import annotations

from enum import IntEnum
from typing import Iterable
import numpy as np
from numpy.typing import ArrayLike, NDArray


class Occupancy(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2


class VoxelMap:
    def __init__(self, minimum: ArrayLike, maximum: ArrayLike, resolution: float,
                 default: Occupancy = Occupancy.UNKNOWN) -> None:
        self.minimum = np.asarray(minimum, dtype=float)
        self.maximum = np.asarray(maximum, dtype=float)
        self.resolution = float(resolution)
        if self.minimum.shape != (3,) or self.maximum.shape != (3,):
            raise ValueError("map bounds must be length-3 vectors")
        if self.resolution <= 0 or np.any(self.maximum <= self.minimum):
            raise ValueError("resolution must be positive and maximum > minimum")
        self.shape = tuple(np.ceil((self.maximum - self.minimum) / self.resolution).astype(int))
        self.data = np.full(self.shape, int(default), dtype=np.uint8)

    def copy(self) -> "VoxelMap":
        result = VoxelMap(self.minimum, self.maximum, self.resolution)
        result.data = self.data.copy()
        return result

    def contains_index(self, index: tuple[int, int, int]) -> bool:
        return all(0 <= index[axis] < self.shape[axis] for axis in range(3))

    def world_to_grid(self, point: ArrayLike, *, clip: bool = False) -> tuple[int, int, int]:
        point_array = np.asarray(point, dtype=float)
        index = np.floor((point_array - self.minimum) / self.resolution).astype(int)
        if clip:
            index = np.clip(index, 0, np.asarray(self.shape) - 1)
        result = tuple(int(value) for value in index)
        if not clip and not self.contains_index(result):
            raise IndexError(f"point {point_array.tolist()} is outside map bounds")
        return result

    def grid_to_world(self, index: tuple[int, int, int]) -> NDArray[np.float64]:
        if not self.contains_index(index):
            raise IndexError(f"grid index {index} is outside map")
        return self.minimum + (np.asarray(index, dtype=float) + 0.5) * self.resolution

    def set_index(self, index: tuple[int, int, int], value: Occupancy) -> None:
        if not self.contains_index(index):
            raise IndexError(f"grid index {index} is outside map")
        self.data[index] = int(value)

    def set_world(self, point: ArrayLike, value: Occupancy) -> None:
        self.set_index(self.world_to_grid(point), value)

    def fill_box(self, lower: ArrayLike, upper: ArrayLike,
                 value: Occupancy = Occupancy.OCCUPIED) -> None:
        lower_i = np.floor((np.asarray(lower, dtype=float) - self.minimum) / self.resolution).astype(int)
        upper_i = np.ceil((np.asarray(upper, dtype=float) - self.minimum) / self.resolution).astype(int)
        lower_i = np.clip(lower_i, 0, np.asarray(self.shape))
        upper_i = np.clip(upper_i, 0, np.asarray(self.shape))
        if np.any(upper_i <= lower_i):
            return
        self.data[tuple(slice(int(lo), int(hi)) for lo, hi in zip(lower_i, upper_i))] = int(value)

    def mark_all_free(self) -> None:
        self.data.fill(int(Occupancy.FREE))

    def occupied_indices(self) -> NDArray[np.int64]:
        return np.argwhere(self.data == int(Occupancy.OCCUPIED))

    def occupied_centers(self) -> NDArray[np.float64]:
        indices = self.occupied_indices()
        return self.minimum + (indices.astype(float) + 0.5) * self.resolution

    def update_points(self, points: Iterable[ArrayLike], value: Occupancy = Occupancy.OCCUPIED) -> int:
        updated = 0
        for point in points:
            try:
                self.set_world(point, value)
                updated += 1
            except IndexError:
                continue
        return updated

