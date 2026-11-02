import json
import time
import numpy as np
import pytest
from robotics_tools.control.protocol import encode, decode
from robotics_tools.hand_tracking.pointing import PointingEstimator
from robotics_tools.hand_tracking.tracker import HandState

KEY = b'0123456789abcdef0123456789abcdef'


def test_authenticated_expiring_packets():
    message = dict(time=time.time(), command='EMERGENCY', seq=1, session='challenge')
    packet = encode(KEY, message)
    assert decode(KEY, packet) == message
    with pytest.raises(ValueError):
        decode(b'x'*32, packet)
    with pytest.raises(ValueError):
        decode(KEY, packet[:-1]+b'x')
    with pytest.raises(ValueError):
        decode(KEY, encode(KEY, dict(message, time=time.time()-1)))
    with pytest.raises(ValueError):
        decode(KEY, encode(KEY, dict(message, time=time.time()+1)))


def test_axis_calibration_and_flex_rejection(tmp_path):
    path = tmp_path/'pointing.json'
    path.write_text(json.dumps(dict(frame_id='map-reference', R_wc=[[0,-1,0],[1,0,0],[0,0,1]],landmark_axis_scale=[1,1,1])))
    estimator = PointingEstimator(path)
    landmarks = np.zeros((21,3))
    landmarks[[5,6,7,8]] = [[0,0,0],[1,0,0],[2,0,0],[3,0,0]]
    state = HandState(True, 1., np.zeros(2), 1., np.zeros(3), 0., landmarks)
    assert np.allclose(estimator.update(state), [0,1,0])
    landmarks[8] = [2,1,0]
    assert estimator.update(state) is None
    landmarks[8] = [2,0,0]
    assert estimator.update(state) is None


def test_reflection_is_not_a_reference_rotation(tmp_path):
    path=tmp_path/'bad.json'
    path.write_text(json.dumps(dict(frame_id='x',R_wc=[[-1,0,0],[0,1,0],[0,0,1]],landmark_axis_scale=[1,1,1])))
    with pytest.raises(ValueError):
        PointingEstimator(path)
