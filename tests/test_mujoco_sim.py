from pathlib import Path
import numpy as np
import mujoco

from ece3162.common.types import Path as MotionPath
from ece3162.simulation.mujoco_runner import MujocoKinematicRunner
from ece3162.trajectory.time_parameterization import TrajectoryGenerator


ROOT = Path(__file__).resolve().parents[1]


def test_f450_model_loads_with_stereo_cameras() -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "simulation/worlds/single_obstacle.xml"))
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "f450") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "stereo_left") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "stereo_right") >= 0


def test_headless_follower_reaches_goal_continuously() -> None:
    route = MotionPath(np.array([[0, 0, 0.75], [1, 0, 1.0], [2, 0, 0.75]], float), "test")
    trajectory = TrajectoryGenerator(1.0, 0.5, 0.75, 20).generate(route)
    runner = MujocoKinematicRunner(ROOT / "simulation/worlds/single_obstacle.xml",
                                   trajectory, route)
    initial = runner.data.mocap_pos[runner.mocap_id].copy()
    midpoint = runner.follower.sample(runner.follower.duration / 2).position
    assert not np.allclose(initial, midpoint)
    final = runner.run_headless()
    np.testing.assert_allclose(final, route.waypoints[-1], atol=1e-9)

