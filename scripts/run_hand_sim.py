#!/usr/bin/env python3
"""Hand goal -> A* -> trajectory -> MuJoCo F450 simulation."""

from __future__ import annotations
import argparse
import math
import os
import platform
from pathlib import Path
import sys
import time
import traceback
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

from robotics_tools.common.types import Path as MotionPath
from robotics_tools.hand_tracking.calibration import HandCalibration, HandCalibrationError, default_calibration
from robotics_tools.hand_tracking.display import OpenCVDisplayProcess
from robotics_tools.hand_tracking.goal_mapper import GoalMapper, TrackingMode
from robotics_tools.hand_tracking.tracker import HandState, LatestFrameCamera, MediaPipeHandTracker
from robotics_tools.mapping.obstacle_inflation import inflate_obstacles
from robotics_tools.mapping.world_loader import load_world
from robotics_tools.planning.astar import AStarPlanner
from robotics_tools.planning.collision_checker import CollisionChecker
from robotics_tools.planning.path_utils import shortcut_path
from robotics_tools.simulation.mujoco_runner import MujocoKinematicRunner
from robotics_tools.trajectory.time_parameterization import TrajectoryGenerator
from robotics_tools.utils.config import load_yaml


def args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", choices=("astar", "rrt", "rrt_star"), default="astar")
    parser.add_argument("--world", default="single_obstacle")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--mock", action="store_true", help="synthetic hand input; no webcam")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def synthetic_state(scale: float = 0.28) -> HandState:
    return HandState(True, time.monotonic(), np.array([0.5, 0.5]), scale,
                     np.array([0, 0, 1.]), 0.0, np.empty((0, 3)))


def main() -> int:
    options = args()
    if options.planner != "astar":
        print(f"Planner not implemented yet: {options.planner}", file=sys.stderr)
        return 2
    if platform.system() == "Darwin" and not options.headless and "MJPYTHON_BIN" not in os.environ:
        launcher = Path(sys.executable).with_name("mjpython")
        if not launcher.is_file():
            print(f"mjpython not found: {launcher}", file=sys.stderr)
            return 2
        os.execv(str(launcher), [str(launcher), *sys.argv])
    hand_config = load_yaml(ROOT / "config/hand.yaml")
    planning_config = load_yaml(ROOT / "config/planning.yaml")
    mapping_config = load_yaml(ROOT / "config/mapping.yaml")
    trajectory_config = load_yaml(ROOT / "config/trajectory.yaml")
    calibration_path = ROOT / "calibration_data/hand/hand_calibration.json"
    if options.mock:
        calibration = default_calibration()
    else:
        try:
            calibration = HandCalibration.load(calibration_path)
        except HandCalibrationError as exc:
            print(exc, file=sys.stderr)
            return 2
    world = load_world(ROOT / "simulation/worlds" / options.world / "world.yaml")
    inflated = inflate_obstacles(world.voxel_map, mapping_config["drone_radius"],
                                 mapping_config["safety_margin"])
    planner = AStarPlanner(connectivity=planning_config["connectivity"],
                           heuristic_weight=planning_config["heuristic_weight"],
                           allow_unknown=planning_config["allow_unknown"])
    checker = CollisionChecker(inflated)
    generator = TrajectoryGenerator(trajectory_config["max_xy_velocity"],
                                    trajectory_config["max_z_velocity"],
                                    trajectory_config["max_acceleration"],
                                    trajectory_config["setpoint_rate_hz"])
    hold_path = MotionPath(np.asarray([world.start]), "hold")
    hold_trajectory = generator.generate(hold_path)
    runner = MujocoKinematicRunner(ROOT / "simulation/worlds" / f"{options.world}.xml",
                                   hold_trajectory, hold_path, options.speed)
    mapper = GoalMapper(calibration, hand_config)

    def replan(start: np.ndarray, goal: np.ndarray) -> bool:
        result = planner.plan(start, goal, inflated)
        if not result.success or result.path is None:
            print(f"NO COLLISION-FREE PATH FOUND: {result.message}")
            return False
        route = shortcut_path(result.path, checker)
        runner.replace_trajectory(generator.generate(route), route, options.speed, True)
        print(f"Hand goal {np.round(goal, 2).tolist()} -> A* {result.planning_time*1000:.1f} ms, "
              f"route {route.total_length:.2f} m")
        return True

    if options.mock:
        # Two stable forward intentions: first advances safely, second requests a
        # goal beyond the blocking wall so A* must route around/above it.
        current = world.start.copy()
        for now in (0.0, 0.1, 0.2, 0.3):
            update = mapper.update(synthetic_state(), current, now)
            if update.should_replan and replan(current, update.goal.position):
                current = runner.run_headless()
        for now in (1.1, 1.2):
            update = mapper.update(synthetic_state(), current, now)
            if update.should_replan and replan(current, update.goal.position):
                current = runner.run_headless()
        print(f"Mock hand simulation final position: {np.round(current, 3).tolist()}")
        return 0

    import mujoco
    import mujoco.viewer
    camera = LatestFrameCamera(0)
    tracker = MediaPipeHandTracker(hand_config["smoothing_alpha"])
    display = OpenCVDisplayProcess("Autonomous Drone Navigation Hand Goal",
                                   Path(sys.executable).with_name("python"))
    display.start()
    motion_start = time.monotonic()
    overlay_reported = False
    try:
        with mujoco.viewer.launch_passive(runner.model, runner.data) as viewer:
            while viewer.is_running():
                display.raise_if_failed()
                if display.stop_requested:
                    break
                frame, captured = camera.latest()
                sample = runner.follower.sample(time.monotonic() - motion_start)
                current = sample.position
                if frame is not None:
                    hand = tracker.process(frame, captured)
                    update = mapper.update(hand, current)
                    if update.should_replan and replan(current, update.goal.position):
                        motion_start = time.monotonic()
                        sample = runner.follower.sample(0.0)
                    overlay = tracker.draw(frame, hand, update.direction.value,
                                           update.goal.position, calibrated=True)
                    import cv2
                    cv2.putText(overlay, f"STATE: {update.mode.value}", (15, 115),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 180, 0), 2)
                    if not overlay_reported:
                        print("Hand overlay:", type(overlay), overlay.shape, overlay.dtype,
                              f"C-contiguous={overlay.flags['C_CONTIGUOUS']}", flush=True)
                        overlay_reported = True
                    display.show(overlay)
                    if update.mode is TrackingMode.HOVER:
                        sample.velocity[:] = 0.0
                runner._apply_sample(sample)
                mujoco.mj_step(runner.model, runner.data)
                viewer.sync()
                time.sleep(runner.model.opt.timestep)
        return 0
    finally:
        tracker.close()
        camera.close()
        display.close()


def entrypoint() -> int:
    """Preserve Python exit status through MuJoCo's macOS trampoline."""
    try:
        status = main()
    except KeyboardInterrupt:
        status = 130
    except BaseException:
        traceback.print_exc()
        status = 1
    if "MJPYTHON_BIN" in os.environ:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(status)
    return status


if __name__ == "__main__":
    raise SystemExit(entrypoint())
