#!/usr/bin/env python3
"""Check the host environment without mutating it or installing hardware tools."""

from __future__ import annotations
import importlib.util
import platform
import sys

REQUIRED = ("numpy", "yaml", "matplotlib", "pytest", "mujoco")
OPTIONAL_LATER_PHASES = ("cv2", "mediapipe", "mavsdk")


def main() -> int:
    print(f"Python:   {sys.version.split()[0]}")
    print(f"Platform: {platform.platform()}")
    if sys.version_info < (3, 11):
        print("ERROR: Python 3.11 or newer is required.")
        return 1
    missing = []
    for module in REQUIRED:
        available = importlib.util.find_spec(module) is not None
        print(f"required {module:12} {'OK' if available else 'MISSING'}")
        if not available:
            missing.append(module)
    for module in OPTIONAL_LATER_PHASES:
        available = importlib.util.find_spec(module) is not None
        print(f"later    {module:12} {'available' if available else 'not installed'}")
    if missing:
        print("Create the Conda environment with: conda env create -f environment.yml")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
