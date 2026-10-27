import json
from pathlib import Path
import numpy as np
import pytest

from robotics_tools.hand_tracking.calibration import (HandCalibration, HandCalibrationError,
                                       default_calibration)
from robotics_tools.hand_tracking.display import normalize_display_frame
from robotics_tools.hand_tracking.goal_mapper import GoalMapper, TrackingMode
from robotics_tools.hand_tracking.gestures import HandDirection
from robotics_tools.hand_tracking.smoothing import ExponentialSmoother
from robotics_tools.hand_tracking.tracker import HandState


def config() -> dict:
    return {"dead_zone": 0.1, "stable_frames": 2, "goal_step": 2.0,
            "max_goal_distance": 3.0, "min_altitude": 0.4, "max_altitude": 3.0,
            "goal_change_threshold": 0.2, "replan_cooldown_s": 0.5,
            "hover_timeout_s": 1.5, "invert_x": False, "invert_y": False,
            "invert_z": False}


def state(center=(0.5, 0.5), scale=0.19, detected=True, timestamp=0.0) -> HandState:
    return HandState(detected, timestamp, np.asarray(center, float), scale, np.array([0, 0, 1.]),
                     0.0, np.empty((0, 3)))


def test_exponential_smoothing() -> None:
    smoother = ExponentialSmoother(0.25)
    np.testing.assert_allclose(smoother.update([0, 0]), [0, 0])
    np.testing.assert_allclose(smoother.update([1, 2]), [0.25, 0.5])


def test_missing_calibration_message(tmp_path: Path) -> None:
    with pytest.raises(HandCalibrationError, match="python scripts/calibrate_hand.py"):
        HandCalibration.load(tmp_path / "missing.json")


def test_calibration_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "hand.json"
    default_calibration().save(path)
    assert HandCalibration.load(path) == default_calibration()


def test_dead_zone_stays_neutral() -> None:
    mapper = GoalMapper(default_calibration(), config())
    update = mapper.update(state(center=(0.51, 0.49)), [0, 0, 1], now=0.0)
    update = mapper.update(state(center=(0.51, 0.49)), [0, 0, 1], now=0.1)
    assert update.direction is HandDirection.NEUTRAL
    assert not update.should_replan


def test_stable_forward_generates_goal_not_motor_command() -> None:
    mapper = GoalMapper(default_calibration(), config())
    mapper.update(state(scale=0.28), [0, 0, 1], now=0.0)
    update = mapper.update(state(scale=0.28), [0, 0, 1], now=0.1)
    assert update.direction is HandDirection.FORWARD
    assert update.should_replan
    np.testing.assert_allclose(update.goal.position, [2, 0, 1])


def test_left_and_altitude_saturation() -> None:
    mapper = GoalMapper(default_calibration(), config())
    mapper.update(state(center=(0.2, 0.5)), [0, 0, 2.8], now=0.0)
    left = mapper.update(state(center=(0.2, 0.5)), [0, 0, 2.8], now=0.1)
    assert left.goal.position[1] > 0
    mapper = GoalMapper(default_calibration(), config())
    mapper.update(state(center=(0.5, 0.2)), [0, 0, 2.8], now=0.0)
    up = mapper.update(state(center=(0.5, 0.2)), [0, 0, 2.8], now=0.1)
    assert up.goal.position[2] == pytest.approx(3.0)


def test_hand_loss_holds_then_hovers() -> None:
    mapper = GoalMapper(default_calibration(), config())
    mapper.update(state(), [0, 0, 1], now=10.0)
    held = mapper.update(state(detected=False), [0, 0, 1], now=10.3)
    hover = mapper.update(state(detected=False), [0, 0, 1], now=12.0)
    assert held.mode is TrackingMode.HAND_LOST
    assert hover.mode is TrackingMode.HOVER
    np.testing.assert_allclose(held.goal.position, hover.goal.position)


def test_display_frame_normalization() -> None:
    source = np.zeros((8, 12, 3), dtype=np.float32)[:, ::2]
    source[0, 0] = [300, -1, 127]
    image = normalize_display_frame(source)
    assert image.shape == (8, 6, 3)
    assert image.dtype == np.uint8
    assert image.flags["C_CONTIGUOUS"]
    np.testing.assert_array_equal(image[0, 0], [255, 0, 127])


def test_display_rejects_none_and_non_bgr_images() -> None:
    with pytest.raises(ValueError, match="None"):
        normalize_display_frame(None)
    with pytest.raises(ValueError, match="shape"):
        normalize_display_frame(np.zeros((8, 12), dtype=np.uint8))
