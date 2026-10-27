"""Process-isolated OpenCV display for coexistence with macOS MuJoCo GUI."""

from __future__ import annotations

import multiprocessing as mp
from pathlib import Path
import queue
import traceback

import numpy as np


def normalize_display_frame(frame: np.ndarray) -> np.ndarray:
    """Return a HighGUI-compatible BGR image or reject an invalid overlay."""
    if frame is None:
        raise ValueError("hand overlay is None")
    image = np.asarray(frame)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"hand overlay must have shape (H, W, 3), got {image.shape}")
    if image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(image)


def _display_worker(window_name: str, frames, stop_requested, errors) -> None:
    import cv2

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        while not stop_requested.is_set():
            try:
                frame = frames.get(timeout=0.05)
            except queue.Empty:
                frame = None
            if frame is not None:
                cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                stop_requested.set()
                break
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                stop_requested.set()
                break
    except KeyboardInterrupt:
        stop_requested.set()
    except BaseException:
        errors.put(traceback.format_exc())
        raise
    finally:
        cv2.destroyAllWindows()


class OpenCVDisplayProcess:
    """Show latest frames in a process whose main thread owns Cocoa HighGUI."""

    def __init__(self, window_name: str, python_executable: str | Path) -> None:
        mp.set_executable(str(python_executable))
        context = mp.get_context("spawn")
        self._frames = context.Queue(maxsize=1)
        self._stop_requested = context.Event()
        self._errors = context.Queue(maxsize=1)
        self._process = context.Process(
            target=_display_worker,
            args=(window_name, self._frames, self._stop_requested, self._errors),
            name="opencv-hand-display",
        )

    def start(self) -> None:
        self._process.start()

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested.is_set()

    def show(self, frame: np.ndarray) -> None:
        self.raise_if_failed()
        image = normalize_display_frame(frame)
        try:
            self._frames.put_nowait(image)
        except queue.Full:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frames.put_nowait(image)
            except queue.Full:
                pass

    def raise_if_failed(self) -> None:
        try:
            details = self._errors.get_nowait()
        except queue.Empty:
            details = None
        if details:
            raise RuntimeError(f"OpenCV hand-display process failed:\n{details}")
        if self._process.exitcode not in (None, 0) and not self.stop_requested:
            raise RuntimeError(f"OpenCV hand-display process exited with status "
                               f"{self._process.exitcode}")

    def close(self) -> None:
        self._stop_requested.set()
        if self._process.pid is None:
            return
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        failure = None
        try:
            self.raise_if_failed()
        except BaseException as exc:
            failure = exc
        finally:
            for channel in (self._frames, self._errors):
                channel.close()
                channel.join_thread()
        if failure is not None:
            raise failure
