import numpy as np
import pytest
from mapping.transforms import camera_to_body_flu, enu_to_ned


def test_camera_axes_to_body_flu() -> None:
    transform = camera_to_body_flu()
    points = transform.apply([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    np.testing.assert_allclose(points, [[1, 0, 0], [0, -1, 0], [0, 0, -1]])


def test_transform_inverse_round_trip() -> None:
    transform = camera_to_body_flu([0.1, -0.2, 0.3])
    point = np.array([0.2, 0.5, 1.2])
    np.testing.assert_allclose(transform.inverse().apply(transform.apply(point)), point,
                               atol=1e-12)


def test_enu_ned_is_self_inverse_for_positions() -> None:
    transform = enu_to_ned()
    np.testing.assert_allclose(transform.apply([1, 2, 3]), [2, 1, -3])
    np.testing.assert_allclose(transform.inverse().apply([2, 1, -3]), [1, 2, 3])

