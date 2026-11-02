"""Small HMAC authenticated UDP messages. Shared 256-bit key, no pickle/code input."""
import hashlib
import hmac
import json
import math
from pathlib import Path
import time


def read_key(path):
    key = Path(path).read_bytes().strip()
    if len(key) < 32:
        raise ValueError('operator key must contain at least 32 random bytes')
    return key


def encode(key, message):
    payload = json.dumps(message, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return hmac.new(key, payload, hashlib.sha256).hexdigest().encode() + b'\n' + payload


def decode(key, packet):
    if len(packet) > 8192:
        raise ValueError('oversized packet')
    signature, payload = packet.split(b'\n', 1)
    if not hmac.compare_digest(signature, hmac.new(key, payload, hashlib.sha256).hexdigest().encode()):
        raise ValueError('authentication failed')
    message = json.loads(payload)
    stamp = message['time']
    if not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or abs(time.time() - stamp) > .4:
        raise ValueError('expired message or clock skew; synchronize Mac and Pi clocks')
    return message
