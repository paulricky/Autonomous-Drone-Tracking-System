"""Configuration-space obstacle inflation for a spherical F450 model."""

from __future__ import annotations
import numpy as np
from .voxel_map import Occupancy, VoxelMap


def inflation_offsets(radius_m: float, resolution: float) -> np.ndarray:
    if radius_m < 0:
        raise ValueError("inflation radius cannot be negative")
    cells = int(np.ceil(radius_m / resolution))
    candidates = np.array([(x, y, z) for x in range(-cells, cells + 1)
                           for y in range(-cells, cells + 1)
                           for z in range(-cells, cells + 1)], dtype=int)
    # Euclidean discrete-ball dilation. Occupied voxels already represent their
    # full cell volumes, so adding another half-cell here would silently exceed
    # the configured physical safety radius and can erase valid narrow passages.
    return candidates[np.linalg.norm(candidates * resolution, axis=1) <= radius_m + 1e-12]


def inflate_obstacles(voxel_map: VoxelMap, drone_radius: float,
                      safety_margin: float) -> VoxelMap:
    radius = float(drone_radius) + float(safety_margin)
    if drone_radius < 0 or safety_margin < 0:
        raise ValueError("drone radius and safety margin must be nonnegative")
    inflated = voxel_map.copy()
    occupied = voxel_map.occupied_indices()
    offsets = inflation_offsets(radius, voxel_map.resolution)
    shape = np.asarray(voxel_map.shape)
    for index in occupied:
        targets = index + offsets
        targets = targets[np.all((targets >= 0) & (targets < shape), axis=1)]
        inflated.data[tuple(targets.T)] = int(Occupancy.OCCUPIED)
    return inflated
