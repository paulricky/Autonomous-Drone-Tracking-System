#!/usr/bin/env python3
"""One Mac operator window. UI and network stay independent of MediaPipe worker."""
import argparse
import multiprocessing as mp
from pathlib import Path
import queue
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/robotics_tools'))
from robotics_tools.control.protocol import read_key, encode, decode


def tracking_worker(frames, output, calibration, camera_index):
    from robotics_tools.hand_tracking.tracker import LatestFrameCamera, MediaPipeHandTracker
    from robotics_tools.hand_tracking.pointing import PointingEstimator
    tracker = MediaPipeHandTracker()
    estimator = PointingEstimator(calibration)
    camera = LatestFrameCamera(camera_index)
    previous = 0
    try:
        while True:
            frame, captured = camera.latest()
            if frame is None or captured <= previous:
                time.sleep(.01)
                continue
            previous = captured
            hand = tracker.process(frame, captured)
            direction = estimator.update(hand)
            # Capture time, never completion time: a stalled inference cannot refresh intent.
            sample = (captured, None if direction is None else direction.tolist(), estimator.frame)
            for channel, item in ((output, sample), (frames, tracker.draw(frame, hand, str(sample[1]), calibrated=True))):
                try:
                    channel.put_nowait(item)
                except queue.Full:
                    try:
                        channel.get_nowait()
                    except queue.Empty:
                        pass
    finally:
        tracker.close()
        camera.close()


class Connection:
    def __init__(self, host, port, key):
        self.peer = (socket.gethostbyname(host), port)
        self.key = key
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(.02)
        self.session = None
        self.seq = self.reply_seq = 0
        self.last_reply = 0
        self.status = {}
        self.last_ack = 'No command acknowledgement'
        self.enabled = self.started = self.quitting = False
        self.sample = None
        self.pending = queue.PriorityQueue()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def request(self, command):
        if command == 'ENABLE':
            self.started = True
        if command in ('EMERGENCY', 'LAND', 'HOLD', 'QUIT'):
            self.enabled = False
            self.sample = None
        self.pending.put((0 if command == 'EMERGENCY' else 1, time.monotonic_ns(), command))

    def send(self, command, **extra):
        self.seq += 1
        self.sock.sendto(encode(self.key, dict(command=command, time=time.time(),
                          session=self.session, seq=self.seq, **extra)), self.peer)

    def loop(self):
        last_send = 0
        retry = None
        pending_sequence = -1
        while True:
            try:
                now = time.monotonic()
                if self.started and (not self.session or now-self.last_reply > .6):
                    self.session = None
                    self.enabled = False
                    if now-last_send > .2:
                        self.send('HELLO')
                        last_send = now
                if self.session:
                    try:
                        _, _, command = self.pending.get_nowait()
                        retry = (command, now, now + 2)
                    except queue.Empty:
                        pass
                    if retry and now >= retry[1]:
                        cmd = retry[0]
                        confirmation = self.status.get('confirmation') if cmd == 'TAKEOFF' else None
                        self.send(cmd, confirmation=confirmation)
                        pending_sequence = self.seq
                        retry = (cmd, now+.15, retry[2]) if now < retry[2] else None
                    if now-last_send > .1:
                        sample = self.sample
                        if self.enabled and sample and now-sample[0] < .2:
                            self.send('DIRECTION', direction=sample[1], frame=sample[2], operator_enabled=True)
                        else:
                            self.send('HEARTBEAT')
                        last_send = now
                try:
                    raw, peer = self.sock.recvfrom(8193)
                    m = decode(self.key, raw)
                    if peer != self.peer:
                        continue
                    if self.session and m.get('session') != self.session:
                        continue
                    if self.session and m['seq'] <= self.reply_seq:
                        continue
                    self.session, self.reply_seq = m['session'], m['seq']
                    self.last_reply = time.monotonic()
                    self.status = m
                    result = m.get('result', '')
                    if result != 'heartbeat' and not result.startswith(('intent accepted', 'REJECTED direction')):
                        self.last_ack = result
                    self.enabled = m.get('enabled', False)
                    if retry and m.get('request_seq') == pending_sequence:
                        retry = None
                except socket.timeout:
                    pass
            except (OSError, ValueError, KeyError, TypeError):
                self.enabled = False
                time.sleep(.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=7447)
    parser.add_argument('--key-file', default=str(Path.home()/'.config/drone/operator.key'))
    parser.add_argument('--calibration', default=str(ROOT/'calibration_data/hand/pointing_reference.json'))
    parser.add_argument('--camera', type=int, default=0)
    args = parser.parse_args()
    from robotics_tools.hand_tracking.pointing import PointingEstimator
    PointingEstimator(args.calibration)  # Fail visibly before worker startup.
    import cv2
    import numpy as np
    context = mp.get_context('spawn')
    frames, output = context.Queue(1), context.Queue(1)
    worker = context.Process(target=tracking_worker, args=(frames, output, args.calibration, args.camera), daemon=True)
    worker.start()
    connection = Connection(args.host, args.port, read_key(args.key_file))
    frame = np.zeros((600, 1000, 3), dtype=np.uint8)
    controls = {ord('s'): 'ENABLE', ord('a'): 'ARM', ord('t'): 'PREPARE_TAKEOFF', ord('y'): 'TAKEOFF',
                32: 'HOLD', ord('l'): 'LAND', ord('e'): 'EMERGENCY', ord('d'): 'DISARM', ord('q'): 'QUIT'}
    window = 'Drone operator'
    cv2.namedWindow(window)
    try:
        while True:
            try:
                frame = frames.get_nowait()
            except queue.Empty:
                pass
            try:
                sample = output.get_nowait()
                connection.sample = sample if sample[1] is not None else None
                if sample[1] is None and connection.enabled:
                    connection.request('HOLD')
            except queue.Empty:
                pass
            image = cv2.resize(frame, (1000, 600))
            status = connection.status
            connected = time.monotonic()-connection.last_reply < .5
            drone = status.get('status', {})
            lines = [
                'S enable | A arm | T then Y confirm takeoff | Space hold',
                'L land | E emergency | D disarm on ground | Q safe quit',
                f'Wi-Fi: {"connected" if connected else "disconnected / press S"} | hand authorized: {connection.enabled}',
                f'Ready: {drone.get("ready", False)} | sensors: {drone.get("sensors_available", False)} | localization: {drone.get("localization_valid", False)}',
                f'Controller: {drone.get("connected", False)} | identity: {drone.get("identity_verified", False)} | failsafes: {drone.get("parameters_verified", False)} | pilot: {drone.get("pilot_enabled", False)}',
                f'Armed: {drone.get("armed", "unknown")} | airborne: {drone.get("airborne", "unknown")} | mode: {drone.get("mode", "unknown")}',
                'Navigation/avoidance: '+str(status.get('navigation', 'unavailable')),
                f'Emergency: {drone.get("emergency", False)} | inhibited: {status.get("latched", False)} | landing: {status.get("landing_state", "unknown")}',
                'Controller: '+str(drone.get('detail', 'unavailable')),
                'Blocked: '+str(drone.get('blocked_reasons', 'controller feedback unavailable')),
                'Command: '+connection.last_ack,
                f'Tracking worker: {worker.is_alive()} | quit pending: {connection.quitting}']
            cv2.rectangle(image, (0, 360), (1000, 600), (0, 0, 0), -1)
            for i, line in enumerate(lines):
                cv2.putText(image, line[:150], (8, 375+i*19), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 240, 180), 1)
            cv2.imshow(window, image)
            key = cv2.waitKey(10) & 0xff
            if key in controls:
                command = controls[key]
                if command == 'QUIT':
                    connection.quitting = True
                    connection.started = True
                connection.request(command)
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                connection.quitting = True
                connection.started = True
                connection.request('QUIT')
                cv2.namedWindow(window)
            if connection.quitting and connected and status.get('safe_to_quit'):
                break
    except KeyboardInterrupt:
        # The ordinary close path remains in the UI until ground/disarmed confirmation.
        connection.request('EMERGENCY')
        raise RuntimeError('Operator process interrupted; onboard/controller failsafes apply. Manual recovery required.')
    finally:
        worker.terminate()
        worker.join(timeout=1)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
