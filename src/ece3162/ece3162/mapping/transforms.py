"""Explicit rigid transforms and fixed frame-convention conversions."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike, NDArray

from ece3162.common.types import CoordinateFrame


@dataclass(frozen=True, slots=True)
class RigidTransform:
    rotation: NDArray[np.float64]
    translation: NDArray[np.float64]
    source: CoordinateFrame
    target: CoordinateFrame

    def __post_init__(self) -> None:
        rotation = np.asarray(self.rotation, dtype=float)
        translation = np.asarray(self.translation, dtype=float)
        if rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError("rotation must be 3x3 and translation must be length 3")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7):
            raise ValueError("rotation must be orthonormal")
        if np.linalg.det(rotation) < 0.999999:
            raise ValueError("rotation must be proper (determinant +1)")
        object.__setattr__(self, "rotation", rotation.copy())
        object.__setattr__(self, "translation", translation.copy())

    def apply(self, points: ArrayLike) -> NDArray[np.float64]:
        points_array = np.asarray(points, dtype=float)
        if points_array.shape[-1] != 3:
            raise ValueError("points must have final dimension 3")
        return points_array @ self.rotation.T + self.translation

    def inverse(self) -> "RigidTransform":
        inverse_rotation = self.rotation.T
        return RigidTransform(inverse_rotation, -inverse_rotation @ self.translation,
                              self.target, self.source)

    def then(self, following: "RigidTransform") -> "RigidTransform":
        if self.target != following.source:
            raise ValueError(f"frame mismatch: {self.target} != {following.source}")
        return RigidTransform(following.rotation @ self.rotation,
                              following.rotation @ self.translation + following.translation,
                              self.source, following.target)


def camera_to_body_flu(translation: ArrayLike = (0.0, 0.0, 0.0)) -> RigidTransform:
    """OpenCV camera (+right,+down,+forward) to body (+forward,+left,+up)."""
    rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    return RigidTransform(rotation, np.asarray(translation, dtype=float),
                          CoordinateFrame.CAMERA, CoordinateFrame.BODY_FLU)


def enu_to_ned() -> RigidTransform:
    """ENU (+east,+north,+up) to PX4 NED (+north,+east,+down)."""
    rotation = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
    return RigidTransform(rotation, np.zeros(3), CoordinateFrame.LOCAL_ENU,
                          CoordinateFrame.PX4_NED)

