"""Controller-independent landing interlocks. No MSP or actuator commands.

A future physical adapter must supply firmware-verified capabilities and fresh,
authoritative feedback. The deployed adapter deliberately has none. Test adapters
are confined to tests; a stopped trajectory is never evidence of landing.
"""
from dataclasses import dataclass
from enum import Enum
import math


class LandingState(str, Enum):
    IDLE = "idle"
    LAND_REQUESTED = "land_requested"
    LANDING = "landing"
    LANDED = "landed"
    DISARMED = "disarmed"
    LANDING_FAILED = "landing_failed"


@dataclass(frozen=True)
class LandingFeedback:
    # Adapter-local monotonic receipt time and strictly increasing sample ID.
    stamp: float
    sequence: int
    connected: bool = False
    authoritative: bool = False
    mode_acknowledged: bool = False
    command_rejected: bool = False
    landed: bool = False
    ground_contact_verified: bool = False
    armed: object = None  # Unknown is distinct from False.
    localization_valid: bool = False
    manual_override: bool = False
    detail: str = ""


class UnavailableLandingController:
    identity_verified = protocol_verified = landing_supported = False
    failsafe_verified = manual_recovery_verified = False
    landing_independent_of_localization = False

    def request_land(self):
        return False

    def request_disarm(self):
        return False

    def feedback(self):
        return None

    def handle_fault(self, reason):
        # No universal throttle/disarm/failsafe command exists.
        pass


class LandingCoordinator:
    def __init__(self, controller, cancel_navigation, *, ack_timeout=3.0,
                 landing_timeout=90.0, feedback_age=0.5, disarm_timeout=3.0):
        for value in (ack_timeout, landing_timeout, feedback_age, disarm_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("landing timeouts must be finite and positive")
        self.controller, self.cancel_navigation = controller, cancel_navigation
        self.ack_timeout, self.landing_timeout = ack_timeout, landing_timeout
        self.feedback_age, self.disarm_timeout = feedback_age, disarm_timeout
        self.state = LandingState.IDLE
        self.detail = "physical landing adapter unavailable; no output enabled"
        self.navigation_inhibited = False
        self.started = self.disarm_started = None
        self.latest = None
        self.last_sequence = -1
        self.last_stamp = -math.inf

    def _fail(self, reason):
        if self.state == LandingState.LANDING_FAILED:
            return False
        self.state, self.detail = LandingState.LANDING_FAILED, reason
        # An adapter may invoke only its independently verified controller policy.
        # Unknown firmware never gets a guessed emergency response.
        if self.controller.failsafe_verified:
            try:
                self.controller.handle_fault(reason)
            except Exception:
                self.detail += "; verified fault handler unavailable"
        return False

    def _fresh(self, now):
        f = self.latest
        return (f is not None and f.authoritative and f.connected and
                math.isfinite(now) and math.isfinite(f.stamp) and
                0 <= now - f.stamp <= self.feedback_age)

    def request_land(self, now):
        if self.navigation_inhibited:
            return self.state != LandingState.LANDING_FAILED
        self.navigation_inhibited = True
        self.state, self.detail = LandingState.LAND_REQUESTED, "landing requested; awaiting controller acknowledgement"
        self.started = now
        try:
            self.cancel_navigation()
        except Exception:
            return self._fail("navigation cancellation failed; manual recovery required")
        c = self.controller
        if not math.isfinite(now) or not all((c.identity_verified, c.protocol_verified,
                c.landing_supported, c.failsafe_verified, c.manual_recovery_verified)):
            return self._fail("controller landing, identity/protocol, failsafe or manual recovery unverified; navigation inhibited; use independently verified pilot recovery")
        try:
            if not c.request_land():
                return self._fail("controller rejected LAND request")
        except Exception:
            return self._fail("LAND transport failure")
        return True

    def observe(self, feedback, now):
        if self.state == LandingState.IDLE:
            return
        f = feedback
        if (not math.isfinite(f.stamp) or f.stamp < self.started or f.sequence <= self.last_sequence or
                f.stamp <= self.last_stamp or f.stamp > now):
            self._fail("invalid or replayed landing telemetry")
            return
        self.latest, self.last_sequence, self.last_stamp = f, f.sequence, f.stamp
        if self.state == LandingState.LANDING_FAILED:
            return  # Failure never clears the movement latch or authorizes shutdown.
        if not self._fresh(now):
            self._fail("landing telemetry disconnected, non-authoritative or stale")
        elif f.manual_override:
            self._fail("manual pilot override active; autonomy remains inhibited")
        elif f.command_rejected:
            self._fail("controller rejected landing mode")
        elif not f.localization_valid and not self.controller.landing_independent_of_localization:
            self._fail("localization invalid for verified landing mode")
        elif self.state in (LandingState.LAND_REQUESTED, LandingState.LANDING):
            if not f.mode_acknowledged:
                if self.state == LandingState.LANDING:
                    self._fail("controller exited acknowledged landing mode")
            else:
                self.state, self.detail = LandingState.LANDING, "controller landing mode acknowledged; " + f.detail
                if f.landed or f.ground_contact_verified:
                    self.state, self.detail = LandingState.LANDED, "authoritative ground/landed confirmation received"
                    if f.armed is False:
                        self.state, self.detail = LandingState.DISARMED, "landed and controller disarming confirmed"
        elif self.state in (LandingState.LANDED, LandingState.DISARMED):
            if not (f.landed or f.ground_contact_verified):
                self._fail("ground confirmation lost")
            elif f.armed is False:
                self.state, self.detail = LandingState.DISARMED, "landed and controller disarming confirmed"
            elif self.state == LandingState.DISARMED:
                self._fail("controller armed state changed or unknown")

    def tick(self, now):
        if self.state in (LandingState.IDLE, LandingState.LANDING_FAILED):
            return
        if not math.isfinite(now) or now < self.started:
            self._fail("invalid landing clock")
        elif self.latest is not None and not self._fresh(now):
            self._fail("landing telemetry lost or stale; verified controller failsafe/manual recovery required")
        elif self.state == LandingState.LAND_REQUESTED and now - self.started > self.ack_timeout:
            self._fail("landing acknowledgement timeout")
        elif self.state == LandingState.LANDING and now - self.started > self.landing_timeout:
            self._fail("landing timeout")
        elif self.disarm_started is not None and self.state == LandingState.LANDED and now - self.disarm_started > self.disarm_timeout:
            self._fail("disarm acknowledgement timeout")

    def request_disarm(self, now):
        if not self._fresh(now) or self.state not in (LandingState.LANDED, LandingState.DISARMED):
            return False
        f = self.latest
        if not (f.landed or f.ground_contact_verified):
            return False
        if self.state == LandingState.DISARMED:
            return f.armed is False
        if f.armed is not True:
            return False
        if self.disarm_started is not None:
            return True
        try:
            if not self.controller.request_disarm():
                return self._fail("controller rejected disarm request")
        except Exception:
            return self._fail("disarm transport failure")
        self.disarm_started = now
        self.detail = "ground confirmed; disarm requested, awaiting controller confirmation"
        return True

    def shutdown_ready(self, now, *, authorized=False):
        return (authorized and self.navigation_inhibited and
                self.state == LandingState.DISARMED and self._fresh(now) and
                self.latest.armed is False and
                (self.latest.landed or self.latest.ground_contact_verified))
