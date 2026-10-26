"""Load deterministic algorithm-simulation worlds from YAML."""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from numpy.typing import NDArray

from ece3162.utils.config import load_yaml, require_keys
from .voxel_map import VoxelMap


@dataclass(slots=True)
class SimulationWorld:
    name: str
    voxel_map: VoxelMap
    start: NDArray[np.float64]
    goal: NDArray[np.float64]
    obstacle_boxes: list[tuple[NDArray[np.float64], NDArray[np.float64]]]


def load_world(path: str | Path) -> SimulationWorld:
    config = load_yaml(path)
    require_keys(config, "name", "map_min", "map_max", "resolution", "start", "goal", "obstacles")
    voxel_map = VoxelMap(config["map_min"], config["map_max"], config["resolution"])
    # Ground-truth simulation has fully observed free space. Perception mode
    # will deliberately retain UNKNOWN until sensed.
    voxel_map.mark_all_free()
    boxes = []
    for obstacle in config["obstacles"]:
        lower = np.asarray(obstacle["lower"], dtype=float)
        upper = np.asarray(obstacle["upper"], dtype=float)
        voxel_map.fill_box(lower, upper)
        boxes.append((lower, upper))
    return SimulationWorld(str(config["name"]), voxel_map,
                           np.asarray(config["start"], dtype=float),
                           np.asarray(config["goal"], dtype=float), boxes)

