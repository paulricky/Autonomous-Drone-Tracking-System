#!/usr/bin/env python3
"""Interactively capture a user's comfortable MediaPipe hand workspace."""

from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

from hand_tracking.calibration import HandCalibration
from hand_tracking.tracker import LatestFrameCamera, MediaPipeHandTracker
from utils.config import load_yaml


def main() -> int:
    import cv2
    config = load_yaml(ROOT / "config/hand.yaml")
    camera = LatestFrameCamera(0)
    tracker = MediaPipeHandTracker(config["smoothing_alpha"])
    poses = ["NEUTRAL", "LEFT", "RIGHT", "UP", "DOWN", "FORWARD", "BACK"]
    captured = {}
    try:
        for pose in poses:
            samples = []
            print(f"Hold {pose}; press SPACE to collect 20 stable samples, Q to cancel.")
            collecting = False
            while len(samples) < 20:
                frame, timestamp = camera.latest()
                if frame is None:
                    time.sleep(0.01)
                    continue
                state = tracker.process(frame, timestamp)
                overlay = tracker.draw(frame, state, pose, calibrated=False)
                cv2.putText(overlay, f"{pose}: {len(samples)}/20", (15, 115),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)
                cv2.imshow("Hand calibration", overlay)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    return 1
                if key == 32:
                    collecting = True
                if collecting and state.detected:
                    samples.append([*state.palm_center, state.palm_scale, state.wrist_roll])
            captured[pose] = np.median(np.asarray(samples), axis=0)
        calibration = HandCalibration(
            neutral=captured["NEUTRAL"][:2].tolist(), left=captured["LEFT"][:2].tolist(),
            right=captured["RIGHT"][:2].tolist(), up=captured["UP"][:2].tolist(),
            down=captured["DOWN"][:2].tolist(), forward_scale=float(captured["FORWARD"][2]),
            backward_scale=float(captured["BACK"][2]), wrist_neutral=float(captured["NEUTRAL"][3]),
            wrist_range=max(abs(float(captured["LEFT"][3] - captured["RIGHT"][3])), 0.1))
        output = ROOT / "calibration_data/hand/hand_calibration.json"
        calibration.save(output)
        print(f"Saved hand calibration: {output}")
        return 0
    finally:
        tracker.close()
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
