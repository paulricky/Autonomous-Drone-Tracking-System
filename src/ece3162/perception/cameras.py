"""Bounded camera-source abstractions; physical capture is explicit and optional."""

from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
import time
import numpy as np


@dataclass(slots=True)
class CameraFrame:
    image: np.ndarray
    timestamp: float
    sequence: int
    source_id: str


class CameraSource(Protocol):
    def read(self) -> CameraFrame | None: ...
    def close(self) -> None: ...


class OpenCVCamera:
    def __init__(self, device: str | int, source_id: str, width: int, height: int, fps: float) -> None:
        import cv2
        self.cv2=cv2; self.capture=cv2.VideoCapture(device); self.source_id=source_id; self.sequence=0
        if not self.capture.isOpened(): raise RuntimeError(f"cannot open camera {device!r}")
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,width); self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
        self.capture.set(cv2.CAP_PROP_FPS,fps)
    def read(self) -> CameraFrame | None:
        ok,image=self.capture.read(); timestamp=time.monotonic()
        if not ok: return None
        result=CameraFrame(image,timestamp,self.sequence,self.source_id); self.sequence+=1; return result
    def close(self) -> None: self.capture.release()


class SequenceCamera:
    def __init__(self, images: list[np.ndarray], timestamps: list[float], source_id: str) -> None:
        if len(images)!=len(timestamps): raise ValueError("images and timestamps must match")
        self.images,self.timestamps,self.source_id=images,timestamps,source_id; self.index=0
    def read(self) -> CameraFrame | None:
        if self.index>=len(self.images): return None
        result=CameraFrame(self.images[self.index].copy(),float(self.timestamps[self.index]),self.index,self.source_id)
        self.index+=1; return result
    def close(self) -> None: pass
