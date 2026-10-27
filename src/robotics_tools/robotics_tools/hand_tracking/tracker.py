"""Latest-frame MediaPipe Hands tracker with smoothed geometric features."""

from __future__ import annotations
from dataclasses import dataclass
import math
import threading
import time
from typing import Any
import numpy as np

from .smoothing import ExponentialSmoother


@dataclass(slots=True)
class HandState:
    detected: bool
    timestamp: float
    palm_center: np.ndarray
    palm_scale: float
    palm_normal: np.ndarray
    wrist_roll: float
    landmarks: np.ndarray
    latency_ms: float = 0.0


class LatestFrameCamera:
    """One-element camera buffer: consumers always receive the newest frame."""
    def __init__(self, camera_index: int = 0) -> None:
        import cv2
        self.cv2 = cv2
        self.capture = cv2.VideoCapture(camera_index)
        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open webcam at index {camera_index}")
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._frame = None
        self._timestamp = 0.0
        self._thread = threading.Thread(target=self._loop, daemon=True, name="latest-hand-frame")
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.capture.read()
            if ok:
                with self._lock:
                    self._frame, self._timestamp = frame, time.monotonic()

    def latest(self):
        with self._lock:
            return self._frame, self._timestamp

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.capture.release()


class MediaPipeHandTracker:
    def __init__(self, smoothing_alpha: float = 0.35) -> None:
        try:
            import mediapipe as mp
        except ImportError as exc:
            raise RuntimeError("MediaPipe is not installed. Run: pip install -r requirements.txt") from exc
        self.mp = mp
        self.hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=1,
                                              model_complexity=0,
                                              min_detection_confidence=0.55,
                                              min_tracking_confidence=0.55)
        self.center_filter = ExponentialSmoother(smoothing_alpha)
        self.feature_filter = ExponentialSmoother(smoothing_alpha)

    def process(self, bgr_frame: np.ndarray, capture_timestamp: float | None = None) -> HandState:
        import cv2
        start = time.perf_counter()
        timestamp = time.monotonic()
        result = self.hands.process(cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB))
        if not result.multi_hand_landmarks:
            empty = np.empty((0, 3), dtype=float)
            return HandState(False, timestamp, np.zeros(2), 0.0, np.zeros(3), 0.0,
                             empty, (time.perf_counter() - start) * 1000)
        raw = result.multi_hand_landmarks[0]
        landmarks = np.array([[point.x, point.y, point.z] for point in raw.landmark])
        palm_indices = [0, 5, 9, 13, 17]
        center = self.center_filter.update(landmarks[palm_indices, :2].mean(axis=0))
        scale = float(np.linalg.norm(landmarks[9, :2] - landmarks[0, :2]))
        across = landmarks[17] - landmarks[5]
        along = landmarks[9] - landmarks[0]
        normal = np.cross(across, along)
        normal /= max(float(np.linalg.norm(normal)), 1e-9)
        roll = math.atan2(float(across[1]), float(across[0]))
        filtered = self.feature_filter.update([scale, roll])
        return HandState(True, timestamp, center, float(filtered[0]), normal,
                         float(filtered[1]), landmarks,
                         (time.perf_counter() - start) * 1000)

    def draw(self, frame: np.ndarray, state: HandState, direction: str = "NEUTRAL",
             goal: np.ndarray | None = None, calibrated: bool = False) -> np.ndarray:
        import cv2
        output = frame.copy()
        if state.detected and len(state.landmarks) == 21:
            height, width = output.shape[:2]
            for point in state.landmarks:
                cv2.circle(output, (int(point[0] * width), int(point[1] * height)), 3, (0, 255, 0), -1)
            center = (int(state.palm_center[0] * width), int(state.palm_center[1] * height))
            cv2.circle(output, center, 10, (0, 255, 255), 2)
        cv2.putText(output, f"DIRECTION: {direction}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        cv2.putText(output, f"CALIBRATION: {'OK' if calibrated else 'MISSING'}", (15, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0) if calibrated else (0, 0, 255), 2)
        if goal is not None:
            cv2.putText(output, f"GOAL: {goal[0]:+.2f} {goal[1]:+.2f} {goal[2]:+.2f}",
                        (15, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        return output

    def close(self) -> None:
        self.hands.close()

