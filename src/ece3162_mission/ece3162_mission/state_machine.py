"""Deterministic mission state machine with no ROS dependency."""
from dataclasses import dataclass
from enum import Enum

class MissionState(str, Enum):
    IDLE = "idle"; VALIDATING = "validating"; PLANNING = "planning"
    TRACKING = "tracking"; SUCCEEDED = "succeeded"; FAILED = "failed"; CANCELLED = "cancelled"

@dataclass
class MissionMachine:
    state: MissionState = MissionState.IDLE
    detail: str = ""
    def request(self, localization_valid: bool, map_valid: bool) -> bool:
        if self.state not in {MissionState.IDLE, MissionState.SUCCEEDED, MissionState.FAILED, MissionState.CANCELLED}: return False
        self.state = MissionState.VALIDATING
        if not localization_valid or not map_valid:
            self.fail("localization or map invalid"); return False
        self.state, self.detail = MissionState.PLANNING, ""; return True
    def plan_ready(self) -> None:
        if self.state != MissionState.PLANNING: raise RuntimeError("plan accepted outside planning state")
        self.state = MissionState.TRACKING
    def complete(self) -> None:
        if self.state != MissionState.TRACKING: raise RuntimeError("completion outside tracking state")
        self.state = MissionState.SUCCEEDED
    def fail(self, detail: str) -> None: self.state, self.detail = MissionState.FAILED, detail
    def cancel(self, detail: str = "operator cancellation") -> None: self.state, self.detail = MissionState.CANCELLED, detail
