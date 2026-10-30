"""Deterministic mission state machine with no ROS dependency."""
from dataclasses import dataclass
from enum import Enum

class MissionState(str, Enum):
    IDLE = "idle"; VALIDATING = "validating"; PLANNING = "planning"
    TRACKING = "tracking"; SUCCEEDED = "succeeded"; FAILED = "failed"; CANCELLED = "cancelled"
    LAND_REQUESTED = "land_requested"; LANDING = "landing"; LANDED = "landed"
    DISARMED = "disarmed"; LANDING_FAILED = "landing_failed"

@dataclass
class MissionMachine:
    state: MissionState = MissionState.IDLE
    detail: str = ""
    navigation_inhibited: bool = False
    def request(self, localization_valid: bool, map_valid: bool) -> bool:
        if self.navigation_inhibited: return False
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
    def fail(self, detail: str) -> None:
        if not self.navigation_inhibited: self.state, self.detail = MissionState.FAILED, detail
    def cancel(self, detail: str = "operator cancellation") -> None:
        if not self.navigation_inhibited: self.state, self.detail = MissionState.CANCELLED, detail
