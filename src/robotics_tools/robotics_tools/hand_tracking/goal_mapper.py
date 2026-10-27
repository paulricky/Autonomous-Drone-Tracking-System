"""Convert calibrated hand intent into bounded local-ENU goals."""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import time
import numpy as np

from robotics_tools.common.types import CoordinateFrame, Goal
from .calibration import HandCalibration
from .gestures import HandDirection, StableDirectionDetector
from .tracker import HandState


class TrackingMode(str, Enum):
    HAND_TRACKING = "HAND_TRACKING"
    HAND_LOST = "HAND_LOST"
    HOVER = "HOVER"


@dataclass(slots=True)
class GoalUpdate:
    goal: Goal
    direction: HandDirection
    mode: TrackingMode
    should_replan: bool


class GoalMapper:
    def __init__(self, calibration: HandCalibration, config: dict) -> None:
        self.calibration = calibration
        self.config = config
        self.detector = StableDirectionDetector(config["dead_zone"], config["stable_frames"])
        self.current_goal: Goal | None = None
        self.last_hand_time: float | None = None
        self.last_plan_time = -float("inf")

    def _normalized_offset(self, state: HandState) -> np.ndarray:
        neutral = np.asarray(self.calibration.neutral)
        horizontal_span = max(abs(self.calibration.right[0] - neutral[0]),
                              abs(neutral[0] - self.calibration.left[0]), 1e-6)
        vertical_span = max(abs(self.calibration.down[1] - neutral[1]),
                            abs(neutral[1] - self.calibration.up[1]), 1e-6)
        depth_span = max(self.calibration.forward_scale - self.calibration.backward_scale, 1e-6)
        depth_neutral = 0.5 * (self.calibration.forward_scale + self.calibration.backward_scale)
        offset = np.array([(state.palm_center[0] - neutral[0]) / horizontal_span,
                           (state.palm_center[1] - neutral[1]) / vertical_span,
                           (state.palm_scale - depth_neutral) / (0.5 * depth_span)])
        for axis, key in enumerate(("invert_x", "invert_y", "invert_z")):
            if self.config.get(key, False):
                offset[axis] *= -1
        return np.clip(offset, -1.0, 1.0)

    def update(self, state: HandState, drone_position, now: float | None = None) -> GoalUpdate:
        now = time.monotonic() if now is None else float(now)
        drone = np.asarray(drone_position, dtype=float)
        if self.current_goal is None:
            self.current_goal = Goal(drone.copy(), frame=CoordinateFrame.LOCAL_ENU)
        if not state.detected:
            age = float("inf") if self.last_hand_time is None else now - self.last_hand_time
            mode = TrackingMode.HOVER if age >= self.config["hover_timeout_s"] else TrackingMode.HAND_LOST
            return GoalUpdate(self.current_goal, self.detector.direction, mode, False)
        self.last_hand_time = now
        offset = self._normalized_offset(state)
        direction = self.detector.update(offset)
        distance = float(self.config["goal_step"])
        direction_vectors = {
            HandDirection.NEUTRAL: np.zeros(3), HandDirection.FORWARD: np.array([1, 0, 0]),
            HandDirection.BACK: np.array([-1, 0, 0]), HandDirection.LEFT: np.array([0, 1, 0]),
            HandDirection.RIGHT: np.array([0, -1, 0]), HandDirection.UP: np.array([0, 0, 1]),
            HandDirection.DOWN: np.array([0, 0, -1]),
        }
        candidate = drone + distance * direction_vectors[direction]
        displacement = candidate - drone
        norm = float(np.linalg.norm(displacement))
        if norm > self.config["max_goal_distance"]:
            candidate = drone + displacement * self.config["max_goal_distance"] / norm
        candidate[2] = np.clip(candidate[2], self.config["min_altitude"], self.config["max_altitude"])
        changed = float(np.linalg.norm(candidate - self.current_goal.position)) > self.config["goal_change_threshold"]
        cooldown_ready = now - self.last_plan_time >= self.config["replan_cooldown_s"]
        should_replan = direction != HandDirection.NEUTRAL and changed and cooldown_ready
        if should_replan:
            self.current_goal = Goal(candidate, frame=CoordinateFrame.LOCAL_ENU)
            self.last_plan_time = now
        return GoalUpdate(self.current_goal, direction, TrackingMode.HAND_TRACKING, should_replan)

