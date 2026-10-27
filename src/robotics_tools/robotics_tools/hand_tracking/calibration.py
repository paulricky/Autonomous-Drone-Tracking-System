"""Persisted hand workspace calibration with actionable missing-data errors."""

from __future__ import annotations
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import numpy as np


class HandCalibrationError(RuntimeError):
    pass


@dataclass(slots=True)
class HandCalibration:
    neutral: list[float]
    left: list[float]
    right: list[float]
    up: list[float]
    down: list[float]
    forward_scale: float
    backward_scale: float
    wrist_neutral: float = 0.0
    wrist_range: float = 1.0

    def __post_init__(self) -> None:
        for name in ("neutral", "left", "right", "up", "down"):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (2,) or not np.all(np.isfinite(value)):
                raise HandCalibrationError(f"{name} must contain two finite normalized coordinates")
        if self.forward_scale <= self.backward_scale:
            raise HandCalibrationError("forward_scale must be greater than backward_scale")

    def save(self, path: str | Path) -> None:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as stream:
            json.dump(asdict(self), stream, indent=2)

    @classmethod
    def load(cls, path: str | Path) -> "HandCalibration":
        calibration_path = Path(path)
        if not calibration_path.is_file():
            raise HandCalibrationError(
                "Hand calibration data not found.\n\nRun:\n\npython scripts/calibrate_hand.py")
        try:
            with calibration_path.open("r", encoding="utf-8") as stream:
                return cls(**json.load(stream))
        except (OSError, json.JSONDecodeError, TypeError, KeyError) as exc:
            raise HandCalibrationError(f"Invalid hand calibration file: {calibration_path}: {exc}") from exc


def default_calibration() -> HandCalibration:
    """Conservative normalized defaults for mock/tests, not a user calibration."""
    return HandCalibration([0.5, 0.5], [0.2, 0.5], [0.8, 0.5],
                           [0.5, 0.2], [0.5, 0.8], 0.28, 0.10)

