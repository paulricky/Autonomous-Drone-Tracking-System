"""Operator protocol and mission authorization, independent of planning callbacks."""
import json
import math
import secrets
import socket
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Vector3
from flight_mission.landing import LandingCoordinator, LandingFeedback, LandingState
from robotics_tools.control.protocol import read_key, encode, decode


class Px4LandingController:
    landing_independent_of_localization = True
    landing_supported = True

    def __init__(self, mission):
        self.mission = mission

    @property
    def identity_verified(self):
        return bool(self.mission.fresh() and self.mission.status.get('identity_verified'))

    protocol_verified = property(lambda self: self.identity_verified)
    failsafe_verified = property(lambda self: bool(self.identity_verified and self.mission.status.get('parameters_verified')))
    manual_recovery_verified = property(lambda self: bool(self.failsafe_verified and self.mission.status.get('pilot_enabled')))

    def request_land(self):
        self.mission.op('LAND')
        return True

    def request_disarm(self):
        self.mission.op('DISARM')
        return True

    def handle_fault(self, reason):
        self.mission.op('EMERGENCY')


class OperatorReceiver(Node):
    def __init__(self):
        super().__init__('operator_receiver')
        self.key = read_key(self.declare_parameter('key_file', '').value)
        self.frame = self.declare_parameter('reference_frame_id', '').value
        if not self.frame:
            raise ValueError('commissioned pointing reference_frame_id required')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.declare_parameter('bind_address', '0.0.0.0').value,
                        self.declare_parameter('port', 7447).value))
        self.sock.setblocking(False)
        self.peer = self.session = None
        self.sequence = -1
        self.reply_sequence = 0
        self.request_sequence = -1
        self.last_packet = self.last_direction = self.status_time = 0.
        self.enabled = self.latched = False
        self.quit_pending = False
        self.arm_sent = self.takeoff_sent = False
        self.confirmation = None
        self.status = {'ready': False, 'detail': 'controller feedback unavailable'}
        self.navigation = 'waiting'
        self.operations = self.create_publisher(String, 'flight/operator_request', 1)
        self.permit = self.create_publisher(Bool, 'mission/navigation_enabled', 1)
        self.stop = self.create_publisher(Empty, 'mission/navigation_stop', 1)
        self.direction = self.create_publisher(Vector3, 'mission/direction', 1)
        self.create_subscription(String, 'flight/status', self.on_status, 10)
        self.create_subscription(String, 'mission/status', self.on_navigation, 10)
        self.landing = LandingCoordinator(Px4LandingController(self), lambda: self.stop.publish(Empty()))
        self.feedback_sequence = 0
        self.create_timer(.02, self.poll)

    def on_navigation(self, msg):
        self.navigation = msg.data

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_time = time.monotonic()
            self.feedback_sequence += 1
            if self.landing.state != LandingState.IDLE:
                s = self.status
                self.landing.observe(LandingFeedback(stamp=self.status_time, sequence=self.feedback_sequence,
                    connected=bool(s.get('connected')), authoritative=bool(s.get('identity_verified')),
                    mode_acknowledged=s.get('mode') == 'AUTO.LAND',
                    landed=bool(s.get('ground')), armed=s.get('armed'),
                    localization_valid=bool(s.get('localization_valid'))), self.status_time)
        except ValueError:
            pass

    def fresh(self):
        return time.monotonic() - self.status_time < .4 and self.status.get('connected')

    def op(self, name):
        self.operations.publish(String(data=name))

    def inhibit(self, name):
        self.enabled = False
        self.last_direction = 0
        self.permit.publish(Bool(data=False))
        if name in ('LAND', 'EMERGENCY'):
            self.latched = True
            self.stop.publish(Empty())
        if name == 'LAND':
            self.landing.request_land(time.monotonic())
        else:
            self.op(name)

    def reply(self, result):
        if not self.peer:
            return
        self.reply_sequence += 1
        status = dict(self.status) if self.fresh() else {'ready': False, 'detail': 'controller feedback stale'}
        response = dict(time=time.time(), session=self.session, seq=self.reply_sequence,
                        result=result, request_seq=self.request_sequence, status=status, navigation=self.navigation,
                        enabled=self.enabled, latched=self.latched,
                        landing_state=self.landing.state.value, landing_detail=self.landing.detail,
                        confirmation=self.confirmation[0] if self.confirmation else None,
                        safe_to_quit=bool(self.fresh() and status.get('identity_verified') and status.get('ground') and status.get('armed') is False))
        try:
            self.sock.sendto(encode(self.key, response), self.peer)
        except OSError:
            pass

    def handle_packet(self, m):
        command = m.get('command')
        if command == 'EMERGENCY':
            self.inhibit('EMERGENCY')
            return 'EMERGENCY latched; controller feedback pending'
        if command == 'LAND':
            self.inhibit('LAND')
            return 'LAND requested; controller feedback pending'
        if command == 'HOLD':
            self.inhibit('HOLD')
            return 'HOLD requested; controller feedback pending'
        if command == 'QUIT':
            self.quit_pending = True
            if not (self.fresh() and self.status.get('ground') and not self.status.get('armed')):
                self.inhibit('LAND')
            return 'QUIT pending authoritative ground and disarmed feedback'
        if command == 'ENABLE':
            if self.latched or self.quit_pending:
                return 'REJECTED: landing/emergency latch; deliberate ground restart required'
            self.enabled = True
            return 'operator enabled; motors unchanged'
        if command == 'DISARM':
            if self.fresh() and self.status.get('ground'):
                if self.landing.state != LandingState.IDLE:
                    if not self.landing.request_disarm(time.monotonic()):
                        return 'REJECTED: landing coordinator ground interlock'
                else:
                    self.op('DISARM')
                return 'disarm requested; awaiting controller feedback'
            return 'REJECTED: ground not confirmed'
        if command in ('ARM', 'PREPARE_TAKEOFF', 'TAKEOFF'):
            if not (self.fresh() and self.enabled and not self.latched and self.status.get('ready')):
                return 'REJECTED: authorization/readiness incomplete'
            if command == 'ARM' and not self.arm_sent and not self.status.get('armed'):
                self.arm_sent = True
                self.op('ARM')
                return 'arm requested; awaiting controller feedback'
            if command == 'PREPARE_TAKEOFF' and self.status.get('armed') and not self.takeoff_sent and self.status.get('ground'):
                self.confirmation = (secrets.token_hex(16), time.monotonic() + 4)
                return 'press Y within four seconds to confirm takeoff'
            if command == 'TAKEOFF' and self.confirmation and m.get('confirmation') == self.confirmation[0] and time.monotonic() < self.confirmation[1] and self.status.get('armed') and not self.takeoff_sent:
                self.confirmation = None
                self.takeoff_sent = True
                self.op('TAKEOFF')
                return 'takeoff requested; awaiting controller flight feedback'
            return 'REJECTED: invalid/repeated transition or missing confirmation'
        if command == 'DIRECTION':
            d = m.get('direction', [])
            if (not self.enabled or self.latched or not self.fresh() or not self.status.get('ready') or
                    not self.status.get('airborne') or self.status.get('mode') == 'AUTO.TAKEOFF' or
                    m.get('frame') != self.frame or m.get('operator_enabled') is not True):
                return 'REJECTED direction: authorization, frame or flight state'
            if len(d) != 3 or any(type(v) not in (float, int) or not math.isfinite(v) for v in d) or abs(sum(v*v for v in d)-1) > .02:
                return 'REJECTED invalid direction'
            self.last_direction = time.monotonic()
            self.direction.publish(Vector3(x=float(d[0]), y=float(d[1]), z=float(d[2])))
            return 'intent accepted; onboard planner retains movement authority'
        return 'heartbeat'

    def poll(self):
        packets = []
        for _ in range(64):
            try:
                raw, peer = self.sock.recvfrom(8193)
                m = decode(self.key, raw)
                if m.get('command') == 'HELLO':
                    if self.peer == peer and time.monotonic()-self.last_packet < .7:
                        self.reply('connected')
                    elif time.monotonic()-self.last_packet > .7:
                        self.peer, self.session = peer, secrets.token_hex(32)
                        self.sequence = -1
                        self.enabled = False
                        self.reply('connected; press S to enable')
                    continue
                if peer != self.peer or m.get('session') != self.session or type(m.get('seq')) is not int or m['seq'] <= self.sequence:
                    continue
                packets.append(m)
            except BlockingIOError:
                break
            except (ValueError, KeyError, TypeError, OSError):
                continue
        # Emergency wins over every other command in this batch; discard older work.
        emergencies = [m for m in packets if m.get('command') == 'EMERGENCY']
        packets = [max(emergencies, key=lambda m: m['seq'])] if emergencies else sorted(packets, key=lambda m: m['seq'])
        for m in packets:
            if m['seq'] <= self.sequence:
                continue
            self.sequence = m['seq']
            self.request_sequence = m['seq']
            self.last_packet = time.monotonic()
            try:
                result = self.handle_packet(m)
            except (ValueError, TypeError):
                result = 'REJECTED malformed command'
            self.reply(result)
        now = time.monotonic()
        self.landing.tick(now)
        if now-self.last_packet > .6 and not self.latched and (self.enabled or self.status.get('armed')):
            self.inhibit('EMERGENCY')
        if self.enabled and self.last_direction and now-self.last_direction > .3:
            self.inhibit('HOLD')
        allowed = self.enabled and not self.latched and now-self.last_direction < .3
        self.permit.publish(Bool(data=allowed))


def main():
    rclpy.init()
    node = OperatorReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()
