"""Exercise the compatibility CLI against the native UDP/ROS safety interlocks."""
import json
import os
import signal
import socket
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from std_msgs.msg import String
from robotics_tools.control.protocol import encode, decode


def test_real_socket_auth_replay_and_emergency_latch(tmp_path):
    key = b"0123456789abcdef0123456789abcdef"
    key_path = tmp_path / "operator.key"
    key_path.write_bytes(key)
    key_path.chmod(0o600)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    namespace = "/operator_compat_test_" + str(os.getpid())
    log_path = tmp_path / "receiver.log"
    log = log_path.open("w")
    process = subprocess.Popen(
        ["ros2", "run", "flight_mission", "operator_receiver", "--ros-args",
         "-r", "__ns:=" + namespace, "-p", "key_file:=" + str(key_path),
         "-p", "reference_frame_id:=measured", "-p", "port:=" + str(port),
         "-p", "bind_address:=127.0.0.1"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node("operator_compat_test", namespace=namespace)
    publisher = node.create_publisher(String, "flight/status", 10)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(.02)
    peer = ("127.0.0.1", port)

    def spin(duration=.05):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def packet(command, seq=0, session=None, **extra):
        return encode(key, dict(time=time.time(), command=command, seq=seq,
                                session=session, **extra))

    def send(command, seq=0, session=None, **extra):
        client.sendto(packet(command, seq, session, **extra), peer)
        end = time.monotonic() + .4
        while time.monotonic() < end:
            spin(.01)
            try:
                response = decode(key, client.recv(8192))
            except socket.timeout:
                continue
            if command == "HELLO" or response["request_seq"] == seq:
                return response
        raise AssertionError("native acknowledgement timed out:\n" + log_path.read_text())

    try:
        end = time.monotonic() + 10
        while True:
            try:
                hello = send("HELLO")
                break
            except AssertionError:
                if process.poll() is not None or time.monotonic() >= end:
                    raise
        assert "Native authenticated operator gateway" in log_path.read_text()
        session = hello["session"]
        assert send("ENABLE", 1, session)["enabled"]
        assert "REJECTED" in send("ARM", 2, session)["result"]
        assert "REJECTED" in send("DIRECTION", 3, session, frame="measured",
                                  direction=[1, 0, 0], operator_enabled=True)["result"]
        emergency = send("EMERGENCY", 4, session)
        assert emergency["latched"] and not emergency["enabled"]
        assert "REJECTED" in send("ENABLE", 5, session)["result"]
        client.sendto(packet("ENABLE", 5, session), peer)
        with pytest.raises(socket.timeout):
            client.recv(8192)
        # Stop the process group so both packets reach the same native poll batch.
        os.killpg(process.pid, signal.SIGSTOP)
        try:
            for seq, command in ((6, "EMERGENCY"), (7, "ENABLE")):
                client.sendto(packet(command, seq, session), peer)
        finally:
            os.killpg(process.pid, signal.SIGCONT)
        client.settimeout(1)
        response = decode(key, client.recv(8192))
        assert response["request_seq"] == 6
        assert response["latched"] and not response["enabled"]
        # The discarded regular command also cannot be replayed after the batch.
        client.settimeout(.1)
        client.sendto(packet("ENABLE", 7, session), peer)
        with pytest.raises(socket.timeout):
            client.recv(8192)
        state = dict(connected=True, ground=True, armed=False, identity_verified=False)
        timer = node.create_timer(.03, lambda: publisher.publish(String(data=json.dumps(state))))
        spin(.3)
        assert not send("HEARTBEAT", 8, session)["safe_to_quit"]
        state["identity_verified"] = True
        spin(.1)
        assert send("HEARTBEAT", 9, session)["safe_to_quit"]
        assert process.poll() is None
    finally:
        client.close()
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
