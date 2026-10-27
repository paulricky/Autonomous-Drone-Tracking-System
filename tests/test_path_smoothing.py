from pathlib import Path
from robotics_tools.mapping.obstacle_inflation import inflate_obstacles
from robotics_tools.mapping.world_loader import load_world
from robotics_tools.planning.astar import AStarPlanner
from robotics_tools.planning.collision_checker import CollisionChecker
from robotics_tools.planning.path_utils import shortcut_path


ROOT = Path(__file__).resolve().parents[1]


def test_shortcut_is_shorter_and_collision_free() -> None:
    world = load_world(ROOT / "simulation/worlds/single_obstacle/world.yaml")
    inflated = inflate_obstacles(world.voxel_map, 0.28, 0.22)
    result = AStarPlanner().plan(world.start, world.goal, inflated)
    assert result.path is not None
    checker = CollisionChecker(inflated)
    smooth = shortcut_path(result.path, checker)
    assert smooth.total_length <= result.path.total_length + 1e-12
    assert len(smooth.waypoints) <= len(result.path.waypoints)
    assert not checker.path_in_collision(smooth)

