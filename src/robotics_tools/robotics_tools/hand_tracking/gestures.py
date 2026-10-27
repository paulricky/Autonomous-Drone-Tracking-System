"""Dead-zoned and temporally stable directional intent."""

from __future__ import annotations
from enum import Enum
import numpy as np


class HandDirection(str, Enum):
    NEUTRAL = "NEUTRAL"
    FORWARD = "FORWARD"
    BACK = "BACK"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    UP = "UP"
    DOWN = "DOWN"


class StableDirectionDetector:
    def __init__(self, dead_zone: float, stable_frames: int) -> None:
        if dead_zone < 0 or stable_frames < 1:
            raise ValueError("dead zone must be nonnegative and stable_frames >= 1")
        self.dead_zone = float(dead_zone)
        self.stable_frames = int(stable_frames)
        self._candidate = HandDirection.NEUTRAL
        self._count = 0
        self.direction = HandDirection.NEUTRAL

    def update(self, normalized_offset: np.ndarray) -> HandDirection:
        x, y, depth = np.asarray(normalized_offset, dtype=float)
        components = np.array([depth, -depth, -x, x, -y, y])
        directions = [HandDirection.FORWARD, HandDirection.BACK, HandDirection.LEFT,
                      HandDirection.RIGHT, HandDirection.UP, HandDirection.DOWN]
        index = int(np.argmax(components))
        candidate = directions[index] if components[index] > self.dead_zone else HandDirection.NEUTRAL
        if candidate == self._candidate:
            self._count += 1
        else:
            self._candidate, self._count = candidate, 1
        if self._count >= self.stable_frames:
            self.direction = candidate
        return self.direction

