import numpy as np
from robotics_tools.common.types import Path
from robotics_tools.mapping.voxel_map import Occupancy, VoxelMap
from robotics_tools.planning.collision_checker import CollisionChecker


def obstacle_map() -> VoxelMap:
    voxel_map = VoxelMap([0, 0, 0], [5, 5, 5], 0.25, Occupancy.FREE)
    voxel_map.fill_box([2, 1, 1], [3, 4, 4])
    return voxel_map


def test_point_and_outside_collisions() -> None:
    checker = CollisionChecker(obstacle_map())
    assert checker.point_in_collision([2.5, 2.0, 2.0])
    assert not checker.point_in_collision([1.0, 2.0, 2.0])
    assert checker.point_in_collision([-1.0, 2.0, 2.0])


def test_segment_checks_interior_not_only_endpoints() -> None:
    checker = CollisionChecker(obstacle_map())
    assert checker.segment_in_collision([1, 2, 2], [4, 2, 2])
    assert not checker.segment_in_collision([1, 0.5, 2], [4, 0.5, 2])


def test_path_checks_every_edge() -> None:
    checker = CollisionChecker(obstacle_map())
    colliding = Path(np.array([[1, 2, 2], [4, 2, 2]]), "test")
    safe = Path(np.array([[1, 0.5, 2], [4, 0.5, 2]]), "test")
    assert checker.path_in_collision(colliding)
    assert not checker.path_in_collision(safe)


def test_unknown_policy_is_explicit() -> None:
    voxel_map = VoxelMap([0, 0, 0], [1, 1, 1], 0.25)
    assert CollisionChecker(voxel_map, unknown_is_occupied=True).point_in_collision([0.5] * 3)
    assert not CollisionChecker(voxel_map, unknown_is_occupied=False).point_in_collision([0.5] * 3)

