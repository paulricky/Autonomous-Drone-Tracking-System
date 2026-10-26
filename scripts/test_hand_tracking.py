#!/usr/bin/env python3
"""Display smoothed hand landmarks, stable direction, and generated goal."""

from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

from ece3162.hand_tracking.calibration import HandCalibration, HandCalibrationError
from ece3162.hand_tracking.goal_mapper import GoalMapper
from ece3162.hand_tracking.tracker import LatestFrameCamera, MediaPipeHandTracker
from ece3162.utils.config import load_yaml


def main() -> int:
    import cv2
    config = load_yaml(ROOT / "config/hand.yaml")
    calibration_path = ROOT / "calibration_data/hand/hand_calibration.json"
    try:
        calibration = HandCalibration.load(calibration_path)
    except HandCalibrationError as exc:
        print(exc, file=sys.stderr)
        return 2
    camera = LatestFrameCamera(0)
    tracker = MediaPipeHandTracker(config["smoothing_alpha"])
    mapper = GoalMapper(calibration, config)
    drone = np.array([0.0, 0.0, 1.0])
    try:
        while True:
            frame, captured = camera.latest()
            if frame is None:
                time.sleep(0.01)
                continue
            state = tracker.process(frame, captured)
            update = mapper.update(state, drone)
            overlay = tracker.draw(frame, state, update.direction.value,
                                   update.goal.position, calibrated=True)
            cv2.putText(overlay, f"STATE: {update.mode.value}  latency={state.latency_ms:.1f} ms",
                        (15, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 2)
            cv2.imshow("ECE3162 Hand Goal Interface", overlay)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        return 0
    finally:
        tracker.close()
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
