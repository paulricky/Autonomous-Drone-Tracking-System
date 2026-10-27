import numpy as np
from robotics_tools.mapping.obstacle_inflation import inflate_obstacles
from robotics_tools.mapping.voxel_map import Occupancy, VoxelMap


def test_inflation_preserves_raw_map_and_expands_obstacle() -> None:
    raw = VoxelMap([0, 0, 0], [5, 5, 5], 0.25, Occupancy.FREE)
    raw.set_world([2.1, 2.1, 2.1], Occupancy.OCCUPIED)
    inflated = inflate_obstacles(raw, 0.3, 0.2)
    assert np.count_nonzero(raw.data == Occupancy.OCCUPIED) == 1
    assert np.count_nonzero(inflated.data == Occupancy.OCCUPIED) > 1
    assert inflated.data[inflated.world_to_grid([2.55, 2.1, 2.1])] == Occupancy.OCCUPIED


def test_zero_inflation_keeps_obstacle() -> None:
    raw = VoxelMap([0, 0, 0], [2, 2, 2], 0.5, Occupancy.FREE)
    raw.set_world([1.25, 1.25, 1.25], Occupancy.OCCUPIED)
    inflated = inflate_obstacles(raw, 0.0, 0.0)
    assert np.array_equal(raw.data, inflated.data)

