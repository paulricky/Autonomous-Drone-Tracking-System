"""Small stateful filters used before hand intent reaches planning."""

from __future__ import annotations
import numpy as np


class ExponentialSmoother:
    def __init__(self, alpha: float) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("smoothing alpha must be in (0, 1]")
        self.alpha = float(alpha)
        self.value: np.ndarray | None = None

    def update(self, value) -> np.ndarray:
        sample = np.asarray(value, dtype=float)
        if not np.all(np.isfinite(sample)):
            raise ValueError("smoothing sample must be finite")
        if self.value is None:
            self.value = sample.copy()
        else:
            self.value = self.alpha * sample + (1.0 - self.alpha) * self.value
        return self.value.copy()

    def reset(self) -> None:
        self.value = None

