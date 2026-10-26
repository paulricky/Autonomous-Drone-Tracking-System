import math
import numpy as np
import pytest

from ece3162.integration.drone_bridge import DuckiedroneBridge, MemoryPublisher
from ece3162.integration.frames import GridFrame2D, planar_path_to_xyz
from ece3162.integration.waypoint_manager import MissionState, WaypointConfig, WaypointManager
from ece3162.mapping.obstacle_inflation import inflate_obstacles
from ece3162.mapping.voxel_map import VoxelMap
from ece3162.planning.astar import AStarPlanner
from ece3162.planning.collision_checker import CollisionChecker
from ece3162.simulation.waypoint_closed_loop import ManualClock, PointMassConfig, run_closed_loop


def test_grid_world_nonzero_origin_resolution_and_rotation():
    frame = GridFrame2D((10.0, -2.0), 0.5, math.pi / 2)
    world = frame.grid_to_world((1, 2))
    assert world == pytest.approx((9.25, -0.75))
    assert frame.world_to_grid(world) == (1, 2)


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan")])
def test_grid_rejects_invalid_resolution(bad):
    with pytest.raises(ValueError): GridFrame2D((0, 0), bad)


def test_path_validation_2d_3d_empty_nonfinite_and_duplicates():
    assert planar_path_to_xyz([[0, 0], [1, 2]], 0.7)[:, 2] == pytest.approx((0.7, 0.7))
    assert planar_path_to_xyz([[0, 0, 1]], 9).shape == (1, 3)
    for bad in ([], [[0]], [[0, np.nan]]):
        with pytest.raises(ValueError): planar_path_to_xyz(bad, 1.0)
    manager = WaypointManager()
    manager.load([[0, 0], [0, 0], [1, 0]])
    assert len(manager.waypoints) == 2


def test_waypoint_advancement_dwell_final_completion_and_initial_pose():
    clock = ManualClock()
    manager = WaypointManager(WaypointConfig(dwell_time=0.2), clock)
    manager.load([[0, 0, 1], [1, 0, 1]])
    manager.start()
    assert manager.update([0, 0, 1], 0) == MissionState.ACTIVE
    clock.now = 0.2
    manager.update([0, 0, 1], clock.now)
    assert manager.index == 1
    clock.now = 1.0
    manager.update([1.05, 0, 1.07], clock.now)
    clock.now = 1.2
    assert manager.update([1.05, 0, 1.07], clock.now) == MissionState.COMPLETE


def test_vertical_tolerance_and_stale_missing_feedback():
    clock = ManualClock()
    manager = WaypointManager(WaypointConfig(dwell_time=0, state_timeout=0.2), clock)
    manager.load([[0, 0, 1]])
    manager.start(); manager.update([0, 0, 1.2], 0)
    assert manager.state == MissionState.ACTIVE
    clock.now = 0.3
    assert manager.update([0, 0, 1], 0) == MissionState.FAILED
    manager.load([[0, 0, 1]]); manager.start()
    assert manager.update(None, None) == MissionState.FAILED


def test_timeout_pause_resume_and_cancel():
    clock = ManualClock()
    manager = WaypointManager(WaypointConfig(dwell_time=0, waypoint_timeout=1, mission_timeout=5), clock)
    manager.load([[2, 0, 1]]); manager.start(); manager.pause()
    clock.now = 10; manager.resume()
    assert manager.update([0, 0, 1], clock.now) == MissionState.ACTIVE
    manager.cancel("emergency")
    assert manager.state == MissionState.CANCELLED and manager.failure_reason == "emergency"
    manager.load([[2, 0, 1]]); manager.start(); clock.now = 11.1
    assert manager.update([0, 0, 1], clock.now) == MissionState.FAILED


def test_bridge_hardware_disabled_default_rate_and_stale_gate():
    publisher = MemoryPublisher()
    bridge = DuckiedroneBridge(publisher)
    assert not bridge.hardware_enabled and not bridge.active
    with pytest.raises(PermissionError): bridge.activate(preflight_ok=True, state_fresh=True)
    enabled = DuckiedroneBridge(publisher, hardware_enabled=True, max_command_rate_hz=10)
    enabled.activate(preflight_ok=True, state_fresh=True)
    assert enabled.publish_waypoint([1, 2, 3], 0, state_fresh=True)
    assert not enabled.publish_waypoint([1, 2, 3], .05, state_fresh=True)
    assert not enabled.publish_waypoint([1, 2, 3], .2, state_fresh=False)
    assert not enabled.active and len(publisher.commands) == 1


def test_astar_to_waypoints_closed_loop_and_clearance():
    world = VoxelMap((0, 0, 0), (5, 5, 3), .25)
    world.mark_all_free(); world.fill_box((2, 0, 0), (3, 3.5, 3))
    inflated = inflate_obstacles(world, .2, .1)
    planned = AStarPlanner(connectivity=26).plan((.5, .5, 1), (4.5, 4.5, 1), inflated)
    assert planned.success and planned.path is not None
    assert not CollisionChecker(inflated).path_in_collision(planned.path)
    clock = ManualClock()
    manager = WaypointManager(WaypointConfig(dwell_time=.05, waypoint_timeout=20), clock)
    manager.load(planned.path.waypoints)
    result = run_closed_loop(manager, np.array([.5, .5, 1]), PointMassConfig(max_time=60))
    assert result.status == MissionState.COMPLETE
    assert np.linalg.norm(result.positions[-1] - [4.5, 4.5, 1]) < .2


def test_unreachable_path_and_failure_scenario():
    world = VoxelMap((0, 0, 0), (3, 3, 2), .25); world.mark_all_free()
    world.fill_box((1.25, 0, 0), (1.75, 3, 2))
    result = AStarPlanner().plan((.5, 1.5, 1), (2.5, 1.5, 1), world)
    assert not result.success and result.path is None
