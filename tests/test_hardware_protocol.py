import pytest
from ece3162.hardware.msp import encode_msp,encode_raw_rc,MSP_SET_RAW_RC
from ece3162.control.pid import PID,PIDConfig

def test_msp_packet_and_rc_bounds():
    packet=encode_raw_rc([1500,1500,1000,900])
    assert packet[:5]==b"$M<\x08"+bytes([MSP_SET_RAW_RC]);assert len(packet)==14
    assert packet[-1]==__import__('functools').reduce(lambda a,b:a^b,packet[3:-1],0)
    with pytest.raises(ValueError):encode_raw_rc([0,1,2,3])

def test_pid_reset_limits_and_validation():
    pid=PID(PIDConfig(2,1,0,minimum=-1,maximum=1,integral_limit=.5))
    assert pid.update(1,.1)==1;pid.reset();assert pid.integral==0
    with pytest.raises(ValueError):pid.update(1,0)
