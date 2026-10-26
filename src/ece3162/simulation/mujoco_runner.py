"""Stage-A MuJoCo kinematic F450 trajectory execution."""

from __future__ import annotations
import math
from pathlib import Path as FilePath
import time
import numpy as np

from ece3162.common.types import Path, Trajectory
from .kinematic_follower import KinematicTrajectoryFollower


class MujocoKinematicRunner:
    def __init__(self, model_path: str | FilePath, trajectory: Trajectory,
                 route: Path, speed_scale: float = 1.0, show_path: bool = True) -> None:
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("MuJoCo is not installed. Run: pip install -r requirements.txt") from exc
        self.mujoco = mujoco
        self.model_path = FilePath(model_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.follower = KinematicTrajectoryFollower(trajectory, speed_scale)
        self.route = route
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "f450")
        self.mocap_id = int(self.model.body_mocapid[body_id])
        if self.mocap_id < 0:
            raise ValueError("F450 model body must be a mocap body for Stage-A simulation")
        self._configure_markers(show_path)
        self._apply_sample(self.follower.sample(0.0))

    def _site_id(self, name: str) -> int:
        return int(self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_SITE, name))

    def _configure_markers(self, show_path: bool) -> None:
        marker_names = [f"path_{index:02d}" for index in range(8)]
        if show_path and len(self.route.waypoints):
            fractions = np.linspace(0, len(self.route.waypoints) - 1,
                                    min(len(marker_names), len(self.route.waypoints)))
            displayed = self.route.waypoints[np.rint(fractions).astype(int)]
        else:
            displayed = np.empty((0, 3))
        for index, name in enumerate(marker_names):
            site_id = self._site_id(name)
            self.model.site_pos[site_id] = displayed[index] if index < len(displayed) else [0, 0, -10]

    def replace_trajectory(self, trajectory: Trajectory, route: Path,
                           speed_scale: float = 1.0, show_path: bool = True) -> None:
        """Replace the active route without recreating the MuJoCo model/viewer."""
        self.follower = KinematicTrajectoryFollower(trajectory, speed_scale)
        self.route = route
        self._configure_markers(show_path)
        self.model.site_pos[self._site_id("goal_marker")] = route.waypoints[-1]

    def _apply_sample(self, sample) -> None:
        self.data.mocap_pos[self.mocap_id] = sample.position
        half_yaw = 0.5 * sample.yaw
        self.data.mocap_quat[self.mocap_id] = [math.cos(half_yaw), 0, 0, math.sin(half_yaw)]
        target_id = self._site_id("current_target")
        self.model.site_pos[target_id] = sample.position + 0.15 * sample.velocity
        self.mujoco.mj_forward(self.model, self.data)

    def run_headless(self, dt: float | None = None) -> np.ndarray:
        step = float(dt or self.model.opt.timestep)
        elapsed = 0.0
        while elapsed < self.follower.duration + step:
            self._apply_sample(self.follower.sample(elapsed))
            self.mujoco.mj_step(self.model, self.data)
            elapsed += step
        self._apply_sample(self.follower.sample(self.follower.duration))
        return self.data.mocap_pos[self.mocap_id].copy()

    def run_interactive(self) -> np.ndarray:
        import mujoco.viewer
        start_time = time.monotonic()
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                elapsed = time.monotonic() - start_time
                sample = self.follower.sample(elapsed)
                self._apply_sample(sample)
                self.mujoco.mj_step(self.model, self.data)
                viewer.sync()
                if sample.complete:
                    # Leave the goal pose visible briefly, then close cleanly.
                    time.sleep(1.0)
                    break
                time.sleep(max(0.0, self.model.opt.timestep - 0.001))
        return self.data.mocap_pos[self.mocap_id].copy()
