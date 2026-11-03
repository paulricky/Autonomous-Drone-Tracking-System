"""Recorded-trace checks for the disconnect safety contract; ROS is not required."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


spec = importlib.util.spec_from_file_location(
    "ros_validator", Path(__file__).parents[1] / "scripts/validate_ros_integration.py")
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)


def trajectory(time):
    seconds = int(time)
    return SimpleNamespace(trajectory_id=1, header=SimpleNamespace(stamp=SimpleNamespace(
        sec=seconds, nanosec=round((time - seconds) * 1e9))))


@pytest.fixture
def trace():
    zero = np.zeros(3)
    times = np.linspace(4.5, 5.0, 6)
    data = dict(
        images=list(times), maps=[(t, 100) for t in times],
        estimates=[(t, zero, True, False) for t in times],
        controls=[(t, zero, 1, False) for t in times],
        truth=[(4.5, zero, zero), (5.0, np.array([.2, 0, 0]), np.array([1., 0, 0]))],
        trajectories=[trajectory(4.8)], failures=[], action_states=[(4.7, "goal", 2)])
    return data


@pytest.mark.parametrize("missing", [None, "images", "maps", "estimates", "controls", "trajectories", "truth", "action_states"])
def test_fault_requires_observed_healthy_navigation(trace, missing):
    if missing:
        trace[missing].clear()
    assert validator.navigation_ready(trace, 5.0) is (missing is None)
    assert not validator.navigation_ready(trace, 6.0)


@pytest.mark.parametrize("violation", [
    None, "no_health", "no_ack", "no_disconnect", "wrong_abort", "no_failure_status",
    "no_action_abort", "continued_control", "new_trajectory", "moving", "unbounded_stop",
    "no_hold", "continued_images", "early_abort", "late_abort",
])
def test_disconnect_requires_each_part_of_safe_abort(trace, violation):
    fault = dict(healthy=True, acknowledged=True, requested_at=5.05, detected_at=5.1, goal_id="goal")
    terminal = "failed:localization or map unavailable"
    trace["failures"] = [(5.6, "localization or map unavailable")]
    trace["action_states"].append((5.61, "goal", 6))
    trace["controls"].append((5.6, np.array([-2., 0, 0]), 1, True))
    trace["truth"].append((5.6, np.array([.4, 0, 0]), np.array([1., 0, 0])))
    trace["truth"].extend((t, np.array([.65, 0, 0]), np.zeros(3)) for t in np.linspace(6.6, 7.5, 20))
    if violation == "no_health":
        fault["healthy"] = False
    elif violation == "no_ack":
        fault["acknowledged"] = False
    elif violation == "no_disconnect":
        fault.pop("detected_at")
    elif violation == "wrong_abort":
        terminal = "failed:goal in collision"
    elif violation == "no_failure_status":
        trace["failures"].clear()
    elif violation == "no_action_abort":
        trace["action_states"].pop()
    elif violation == "continued_control":
        trace["controls"].append((5.8, np.ones(3), 1, False))
    elif violation == "new_trajectory":
        trace["trajectories"].append(trajectory(6.0))
    elif violation == "moving":
        trace["truth"][-1][2][0] = 1.0
    elif violation == "unbounded_stop":
        trace["truth"][-1][1][0] = 2.0
    elif violation == "no_hold":
        trace["truth"] = trace["truth"][:3]
    elif violation == "continued_images":
        trace["images"].append(5.7)
    elif violation in ("early_abort", "late_abort"):
        trace["failures"][0] = (5.2 if violation == "early_abort" else 6.2, "localization or map unavailable")
    result = validator.disconnect_safety(fault, terminal, trace)
    assert result["passed"] is (violation is None), result
