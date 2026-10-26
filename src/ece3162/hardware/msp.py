"""Small, testable subset of recovered MultiWii Serial Protocol framing.

Derived from the recovered pidrone_pkg h2rMultiWii implementation. Serial I/O
is intentionally outside this module and requires an explicit platform adapter.
"""
from __future__ import annotations
import struct
from collections.abc import Iterable

MSP_SET_RAW_RC=200

def encode_msp(command:int,payload:bytes=b"")->bytes:
    if not 0<=command<=255 or len(payload)>255:raise ValueError("MSP v1 command and payload must fit one byte")
    checksum=len(payload)^command
    for value in payload:checksum^=value
    return b"$M<"+bytes((len(payload),command))+payload+bytes((checksum,))

def encode_raw_rc(channels:Iterable[int])->bytes:
    values=tuple(int(v) for v in channels)
    if len(values)!=4 or any(v<900 or v>2100 for v in values):raise ValueError("four bounded legacy RC channels required")
    return encode_msp(MSP_SET_RAW_RC,struct.pack("<4H",*values))
