"""Local-map wrapper reserved for temporal perception updates."""
from dataclasses import dataclass
from .voxel_map import VoxelMap


@dataclass(slots=True)
class LocalMap:
    voxels: VoxelMap
    timestamp: float

