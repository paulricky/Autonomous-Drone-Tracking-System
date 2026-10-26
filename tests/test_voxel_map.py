import numpy as np
import pytest
from ece3162.mapping.voxel_map import Occupancy, VoxelMap


def test_index_world_round_trip() -> None:
    voxel_map = VoxelMap([-1, -1, 0], [1, 1, 2], 0.5)
    for index in ((0, 0, 0), (3, 3, 3), (2, 1, 2)):
        assert voxel_map.world_to_grid(voxel_map.grid_to_world(index)) == index


def test_free_occupied_unknown_states() -> None:
    voxel_map = VoxelMap([0, 0, 0], [2, 2, 2], 1.0)
    assert voxel_map.data[0, 0, 0] == Occupancy.UNKNOWN
    voxel_map.set_world([0.5, 0.5, 0.5], Occupancy.FREE)
    voxel_map.set_world([1.5, 1.5, 1.5], Occupancy.OCCUPIED)
    assert voxel_map.data[0, 0, 0] == Occupancy.FREE
    assert voxel_map.data[1, 1, 1] == Occupancy.OCCUPIED


def test_out_of_bounds_rejected() -> None:
    voxel_map = VoxelMap([0, 0, 0], [1, 1, 1], 0.25)
    with pytest.raises(IndexError):
        voxel_map.world_to_grid([1.0, 0.5, 0.5])

