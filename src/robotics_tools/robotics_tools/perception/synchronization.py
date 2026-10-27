"""Timestamp pairing with bounded queues and diagnostics."""
from __future__ import annotations
from dataclasses import dataclass
from collections import deque
from .cameras import CameraFrame

@dataclass(slots=True)
class StereoPair:
    left: CameraFrame; right: CameraFrame; timestamp_delta: float

class StereoSynchronizer:
    def __init__(self,max_delta:float=.005,max_queue:int=8)->None:
        if max_delta<0 or max_queue<1: raise ValueError("invalid synchronization limits")
        self.max_delta=max_delta; self.left=deque(maxlen=max_queue); self.right=deque(maxlen=max_queue)
        self.paired=0; self.rejected=0
    def add_left(self,frame:CameraFrame)->None:self.left.append(frame)
    def add_right(self,frame:CameraFrame)->None:self.right.append(frame)
    def pop_pair(self)->StereoPair|None:
        while self.left and self.right:
            l,r=self.left[0],self.right[0]; delta=l.timestamp-r.timestamp
            if abs(delta)<=self.max_delta:
                self.left.popleft();self.right.popleft();self.paired+=1;return StereoPair(l,r,abs(delta))
            (self.left if delta<0 else self.right).popleft();self.rejected+=1
        return None
