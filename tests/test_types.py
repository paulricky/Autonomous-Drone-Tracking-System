import numpy as np
import pytest
from ece3162.common.types import CoordinateFrame, DroneState, Path


def test_path_length() -> None:
    path = Path(np.array([[0, 0, 0], [3, 4, 0]], dtype=float), "test")
    assert path.total_length == pytest.approx(5.0)


def test_drone_state_rejects_ambiguous_vector() -> None:
    with pytest.raises(ValueError, match="position"):
        DroneState([1, 2], [0, 0, 0], [0, 0, 0, 1], 0.0)


def test_frame_is_explicit() -> None:
    state = DroneState([1, 2, 3], [0, 0, 0], [0, 0, 0, 1], 0.0)
    assert state.frame is CoordinateFrame.LOCAL_ENU

