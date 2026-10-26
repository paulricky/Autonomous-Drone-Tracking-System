from pathlib import Path
from ece3162.mapping.obstacle_inflation import inflate_obstacles
from ece3162.mapping.world_loader import load_world
from ece3162.planning.astar import AStarPlanner
from ece3162.planning.collision_checker import CollisionChecker


ROOT = Path(__file__).resolve().parents[1]


def test_astar_solves_single_obstacle_in_inflated_space() -> None:
    world = load_world(ROOT / "simulation/worlds/single_obstacle/world.yaml")
    inflated = inflate_obstacles(world.voxel_map, 0.28, 0.22)
    planner = AStarPlanner(connectivity=26)
    result = planner.plan(world.start, world.goal, inflated)
    assert result.success, result.message
    assert result.path is not None
    assert result.expanded_nodes > 0
    assert result.generated_nodes >= result.expanded_nodes
    assert not CollisionChecker(inflated).path_in_collision(result.path)


def test_astar_rejects_goal_in_obstacle() -> None:
    world = load_world(ROOT / "simulation/worlds/single_obstacle/world.yaml")
    planner = AStarPlanner()
    result = planner.plan(world.start, [1.5, 0.0, 1.0], world.voxel_map)
    assert not result.success
    assert "goal is in collision" in result.message


def test_connectivity_validation() -> None:
    try:
        AStarPlanner(connectivity=12)
    except ValueError as exc:
        assert "6, 18, or 26" in str(exc)
    else:
        raise AssertionError("invalid connectivity was accepted")
