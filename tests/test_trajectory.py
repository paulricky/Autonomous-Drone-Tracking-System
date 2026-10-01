import numpy as np
import pytest
from common.types import Path
from trajectory.time_parameterization import TrajectoryGenerator


def generator() -> TrajectoryGenerator:
    return TrajectoryGenerator(1.0, 0.4, 0.6, 50.0)


def test_trajectory_reaches_goal_with_monotonic_time() -> None:
    path = Path(np.array([[0, 0, 1], [2, 0, 1], [2, 1, 1.5]], float), "test")
    trajectory = generator().generate(path)
    np.testing.assert_allclose(trajectory.points[0].position, path.waypoints[0])
    np.testing.assert_allclose(trajectory.points[-1].position, path.waypoints[-1])
    times = np.array([point.time_from_start for point in trajectory.points])
    assert np.all(np.diff(times) > 0)


def test_velocity_and_acceleration_limits() -> None:
    path = Path(np.array([[0, 0, 0], [3, 0, 0], [3, 0, 2]], float), "test")
    trajectory = generator().generate(path)
    for point in trajectory.points:
        assert np.linalg.norm(point.velocity[:2]) <= 1.0 + 1e-9
        assert abs(point.velocity[2]) <= 0.4 + 1e-9
        assert np.linalg.norm(point.acceleration) <= 0.6 + 1e-9


def test_short_segment_uses_triangular_profile() -> None:
    path = Path(np.array([[0, 0, 0], [0.01, 0, 0]], float), "test")
    trajectory = generator().generate(path)
    assert max(np.linalg.norm(point.velocity) for point in trajectory.points) < 1.0
    assert trajectory.points[-1].velocity.tolist() == [0.0, 0.0, 0.0]


def test_invalid_limits_rejected() -> None:
    with pytest.raises(ValueError):
        TrajectoryGenerator(1.0, 0.4, 0.0)
