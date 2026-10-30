"""Run deployed read-only telemetry against a PTY controller, never real USB."""
import os
from pathlib import Path
import platform
import pty
import select
import signal
import subprocess
import threading
import time
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.qos import QoSProfile, DurabilityPolicy
from flight_interfaces.msg import MspControllerIdentity, MspRawTelemetry, FlightControllerStatus


@pytest.mark.skipif(platform.system() != "Linux" or os.geteuid() != 0,
                    reason="persistent serial fixture requires isolated root Linux container")
@pytest.mark.parametrize("profile", ["modern", "legacy", "unsupported_api"])
def test_msp_identity_telemetry_and_disconnect_are_read_only(tmp_path, profile):
    master, slave = pty.openpty()
    path = Path("/dev/serial/by-id") / f"robotics-test-{os.getpid()}-{profile}"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.symlink_to(os.ttyname(slave))
    stop = threading.Event()
    seen, errors = [], []
    status_payload = bytes([232, 3, 0, 0, 1, 0, 0, 0, 0, 0, 0])
    responses = {1: bytes([0, 2 if profile == "unsupported_api" else 1, 46]),
                 2: b"TEST", 3: bytes([4, 5, 1]), 4: b"MOCK\x00\x00",
                 100: bytes([1, 2, 0, 0, 0, 0, 0]),
                 101: status_payload + (b"\x01\x02" if profile == "modern" else b""),
                 110: bytes(9 if profile == "modern" else 7), 102: bytes(18)}

    def peer():
        pending = bytearray()
        while not stop.is_set():
            try:
                if not select.select([master], [], [], .05)[0]: continue
                pending.extend(os.read(master, 1024))
                while len(pending) >= 6:
                    request = bytes(pending[:6]); del pending[:6]
                    command = request[4]
                    if request[:4] != b"$M<\x00" or request[5] != command or command not in responses:
                        errors.append(request)
                        return
                    seen.append(command)
                    rejected = profile == "legacy" and command == 1
                    payload = b"" if rejected else responses[command]
                    checksum = len(payload) ^ command
                    for byte in payload: checksum ^= byte
                    reply = b"$M" + (b"!" if rejected else b">") + bytes([len(payload), command]) + payload + bytes([checksum])
                    os.write(master, reply)
            except OSError:
                if not stop.is_set(): errors.append("virtual peer I/O failure")
                return

    thread = threading.Thread(target=peer, daemon=True)
    namespace = "/msp_test_" + str(os.getpid())
    rclpy.init()
    node = rclpy.create_node("msp_inventory_test", namespace=namespace)
    identities, telemetry, statuses = [], [], []
    node.create_subscription(MspControllerIdentity, "flight_controller/identity", identities.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(MspRawTelemetry, "flight_controller/raw_telemetry", telemetry.append, 10)
    node.create_subscription(FlightControllerStatus, "flight_controller/status", statuses.append, 10)
    log = (tmp_path / "telemetry.log").open("w")
    process = None
    def until(predicate, timeout=10):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert predicate(), (tmp_path / "telemetry.log").read_text()
    try:
        thread.start()
        process = subprocess.Popen(["ros2", "run", "robotics_core", "msp_telemetry_node",
            "--ros-args", "-r", "__ns:=" + namespace,
            "-p", "transport_enabled:=true", "-p", "protocol:=msp_v1_read_only_candidate",
            "-p", "serial_device:=" + str(path)], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        until(lambda: bool(identities and telemetry and statuses))
        identity, raw = identities[-1], telemetry[-1]
        assert not identity.control_interface_verified and not identity.landing_interface_verified
        assert not statuses[-1].hardware_output_enabled and not statuses[-1].command_gate_open
        assert raw.poll_duration_s >= 0 and "UNVERIFIED" in raw.imu_metadata
        if profile == "unsupported_api":
            assert identity.api_major == 2 and not raw.raw_imu_available
            assert seen == [1] and not statuses[-1].connected
        else:
            assert raw.raw_imu_available and statuses[-1].connected
            if profile == "modern":
                assert identity.firmware_variant == "TEST" and identity.board_identifier == "MOCK"
                assert not raw.ident_available and not raw.status_available
                assert len(raw.status_payload) == 13 and len(raw.analog_payload) == 9
                assert 100 not in seen
            else:
                assert not identity.api_available and raw.ident_available and raw.status_available
                assert seen[:2] == [1, 100]
            stop.set(); thread.join(timeout=2)
            os.close(master); master = None
            until(lambda: bool(statuses and not statuses[-1].connected))
        assert not errors
        assert set(seen) <= {1, 2, 3, 4, 100, 101, 102, 110}
        assert process.poll() is None
    finally:
        stop.set(); thread.join(timeout=2)
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
        log.close()
        node.destroy_node(); rclpy.shutdown()
        path.unlink()
        if master is not None: os.close(master)
        os.close(slave)
