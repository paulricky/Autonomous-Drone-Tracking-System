"""Cross-language protocol test against the production C++ gateway, with no controller."""
import os
from pathlib import Path
import socket
import subprocess
import time
import pytest
rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import String, Empty
from robotics_tools.control.protocol import encode, decode
import json


def test_native_gateway_single_t_and_emergency(tmp_path):
    executable = os.environ.get('NATIVE_OPERATOR_EXECUTABLE')
    if not executable:
        pytest.skip('native gateway executable required')
    key = b'0123456789abcdef0123456789abcdef'
    key_file = tmp_path/'operator.key';key_file.write_bytes(key);key_file.chmod(0o600)
    reservation=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);reservation.bind(('127.0.0.1',0))
    port=reservation.getsockname()[1];reservation.close()
    process=subprocess.Popen([executable,'--ros-args','-r','__ns:=/native_operator_test','-p','key_file:='+str(key_file),'-p','reference_frame_id:=measured','-p','bind_address:=127.0.0.1','-p','port:='+str(port)],stdout=subprocess.DEVNULL)
    rclpy.init();node=rclpy.create_node('native_operator_integration',namespace='/native_operator_test')
    state=dict(connected=True,ready=True,identity_verified=True,ground=True,armed=False,airborne=False,mode='POSCTL',takeoff_allowed=True)
    publisher=node.create_publisher(String,'flight/status',10)
    commands=[];emergencies=[]
    sub=node.create_subscription(String,'flight/operator_request',lambda m:commands.append(m.data),10)
    emergency_sub=node.create_subscription(Empty,'flight/emergency_request',lambda m:emergencies.append(True),10)
    timer=node.create_timer(.03,lambda:publisher.publish(String(data=json.dumps(state))))
    client=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);client.settimeout(.02)
    seq=0;session=None
    def spin(duration=.06):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    def send(command,wait=True,**extra):
        nonlocal seq,session
        seq+=1
        packet=encode(key,dict(time=time.time(),command=command,seq=seq,session=session,**extra))
        client.sendto(packet,('127.0.0.1',port))
        if not wait:return
        deadline=time.monotonic()+.4
        while time.monotonic()<deadline:
            spin(.01)
            try:response=decode(key,client.recv(8192))
            except socket.timeout:continue
            if command=='HELLO' or response.get('request_seq')==seq:
                if command=='HELLO':session=response['session']
                return response
        raise AssertionError('no matching native acknowledgement')
    try:
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            try:send('HELLO');break
            except AssertionError:pass
        assert session
        spin(.15)
        assert send('ENABLE')['enabled']
        assert 'REJECTED' in send('TAKEOFF')['result']
        send('ARM');spin(.05)
        state['armed']=True;spin(.06)
        assert 'takeoff requested' in send('TAKEOFF')['result']
        assert 'REJECTED' in send('TAKEOFF')['result']
        assert 'REJECTED' in send('TAKEOFF')['result']
        spin(.05)
        assert commands.count('TAKEOFF')==1
        send('EMERGENCY');spin(.06)
        assert emergencies
        assert send('ENABLE')['latched']
        assert 'REJECTED' in send('TAKEOFF')['result']
        state['armed']=False;spin(.06)
        assert send('QUIT')['safe_to_quit']
    finally:
        client.close();process.terminate();process.wait(timeout=3)
        node.destroy_node();rclpy.shutdown()
