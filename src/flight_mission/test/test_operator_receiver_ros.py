"""Exercise actual UDP/ROS interlocks without a controller or actuator output."""
import json
import socket
import time
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from flight_mission.operator_receiver import OperatorReceiver
from robotics_tools.control.protocol import encode, decode


def test_real_socket_auth_replay_and_emergency_latch(tmp_path):
    key=b'0123456789abcdef0123456789abcdef'
    key_path=tmp_path/'operator.key';key_path.write_bytes(key)
    rclpy.init(args=['--ros-args', '-p', 'key_file:='+str(key_path), '-p', 'reference_frame_id:=measured', '-p', 'port:=0', '-p', 'bind_address:=127.0.0.1'])
    node=OperatorReceiver()
    client=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);client.settimeout(.1)
    peer=node.sock.getsockname()
    def send(command, seq=0, session=None, **extra):
        client.sendto(encode(key,dict(time=time.time(),command=command,seq=seq,session=session,**extra)),peer)
        node.poll()
        return decode(key,client.recv(8192))
    try:
        hello=send('HELLO');session=hello['session']
        assert send('ENABLE',1,session)['enabled']
        assert 'REJECTED' in send('ARM',2,session)['result']
        assert 'REJECTED' in send('DIRECTION',3,session,frame='measured',direction=[1,0,0],operator_enabled=True)['result']
        emergency=send('EMERGENCY',4,session)
        assert emergency['latched'] and not emergency['enabled']
        assert 'REJECTED' in send('ENABLE',5,session)['result']
        client.sendto(encode(key,dict(time=time.time(),command='ENABLE',seq=5,session=session)),peer)
        node.poll()
        with pytest.raises(socket.timeout):
            client.recv(8192)
        assert node.latched
        # Emergency wins even when a newer regular command is in the same batch.
        for seq, command in ((6,'EMERGENCY'),(7,'ENABLE')):
            client.sendto(encode(key,dict(time=time.time(),command=command,seq=seq,session=session)),peer)
        node.poll()
        assert decode(key,client.recv(8192))['latched']
        assert not node.enabled
        node.status = dict(connected=True, ground=True, armed=False, identity_verified=False)
        node.status_time = time.monotonic()
        assert not send('HEARTBEAT', 8, session)['safe_to_quit']
        node.status['identity_verified'] = True
        assert send('HEARTBEAT', 9, session)['safe_to_quit']
    finally:
        client.close();node.sock.close();node.destroy_node();rclpy.shutdown()
