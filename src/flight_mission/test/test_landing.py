"""Mock controller tests; no serial devices, actuator packets or OS shutdown."""
from dataclasses import replace
import pytest
from flight_mission.landing import (LandingCoordinator, LandingFeedback, LandingState,
                                   UnavailableLandingController)
from flight_mission.state_machine import MissionMachine
from flight_mission.readiness import evaluate


class MockController:
    identity_verified = protocol_verified = landing_supported = True
    failsafe_verified = manual_recovery_verified = True
    landing_independent_of_localization = False

    def __init__(self):
        self.land_calls = self.disarm_calls = 0
        self.land_accept = self.disarm_accept = True
        self.faults = []

    def request_land(self):
        self.land_calls += 1
        return self.land_accept

    def request_disarm(self):
        self.disarm_calls += 1
        return self.disarm_accept

    def handle_fault(self, reason): self.faults.append(reason)


def setup():
    controller = MockController()
    mission = MissionMachine()
    assert mission.request(True, True)
    mission.plan_ready()
    cancelled = []
    def cancel():
        mission.navigation_inhibited = True
        cancelled.append(True)
    landing = LandingCoordinator(controller, cancel)
    return landing, controller, mission, cancelled


def feedback(stamp=1.1, sequence=1, **kwargs):
    return replace(LandingFeedback(stamp, sequence, connected=True, authoritative=True,
                   localization_valid=True, armed=True), **kwargs)


def test_request_ack_ground_disarm_shutdown_sequence():
    landing, controller, mission, cancelled = setup()
    assert landing.request_land(1.0)
    assert landing.state == LandingState.LAND_REQUESTED
    assert cancelled == [True]
    assert not mission.request(True, True)
    assert not landing.request_disarm(1.0)
    assert not landing.shutdown_ready(1.0)
    landing.observe(feedback(mode_acknowledged=True), 1.1)
    assert landing.state == LandingState.LANDING
    assert not landing.request_disarm(1.1)
    landing.observe(feedback(1.2, 2, mode_acknowledged=True, landed=True), 1.2)
    assert landing.state == LandingState.LANDED
    assert not landing.shutdown_ready(1.2)
    assert landing.request_disarm(1.2)
    assert landing.request_disarm(1.2)
    assert controller.disarm_calls == 1
    assert landing.state == LandingState.LANDED  # Request is not confirmation.
    landing.observe(feedback(1.3, 3, landed=True, armed=False), 1.3)
    assert landing.state == LandingState.DISARMED
    assert landing.shutdown_ready(1.3)
    assert not landing.shutdown_ready(2.0)


def test_repeated_land_cannot_restart_or_clear_inhibition():
    landing, controller, mission, cancelled = setup()
    assert landing.request_land(1.0)
    assert landing.request_land(2.0)
    assert controller.land_calls == 1 and cancelled == [True] and landing.started == 1.0
    mission.cancel(); mission.fail("late action callback")
    assert not mission.request(True, True)


@pytest.mark.parametrize("missing", ["identity_verified", "protocol_verified", "landing_supported",
                                     "failsafe_verified", "manual_recovery_verified"])
def test_unverified_capability_sends_nothing(missing):
    landing, controller, _, _ = setup()
    setattr(controller, missing, False)
    assert not landing.request_land(1.0)
    assert landing.state == LandingState.LANDING_FAILED
    assert landing.navigation_inhibited and controller.land_calls == 0


def test_production_backend_is_unavailable():
    landing = LandingCoordinator(UnavailableLandingController(), lambda: None)
    assert not landing.request_land(1.0)
    assert not landing.request_land(2.0)
    assert not landing.request_disarm(2.0) and not landing.shutdown_ready(2.0)


@pytest.mark.parametrize("fault", ["command_rejected", "manual_override"])
def test_controller_rejection_or_pilot_override(fault):
    landing, controller, _, _ = setup()
    landing.request_land(1.0)
    landing.observe(feedback(**{fault: True}), 1.1)
    assert landing.state == LandingState.LANDING_FAILED and len(controller.faults) == 1
    assert controller.disarm_calls == 0


@pytest.mark.parametrize("changes", [{"connected": False}, {"authoritative": False},
                                     {"localization_valid": False}])
def test_loss_invalid_localization_and_non_authoritative_feedback(changes):
    landing, controller, _, _ = setup()
    landing.request_land(1.0)
    landing.observe(feedback(**changes), 1.1)
    assert landing.state == LandingState.LANDING_FAILED
    assert controller.disarm_calls == 0 and controller.faults


def test_verified_onboard_landing_may_be_independent_of_ros_localization():
    landing, controller, _, _ = setup()
    controller.landing_independent_of_localization = True
    landing.request_land(1.0)
    landing.observe(feedback(mode_acknowledged=True, localization_valid=False), 1.1)
    assert landing.state == LandingState.LANDING


def test_ack_timeout_does_not_disarm():
    landing, controller, _, _ = setup()
    landing.request_land(1.0)
    landing.tick(4.1)
    assert landing.state == LandingState.LANDING_FAILED and controller.disarm_calls == 0


def test_connection_silence_after_ack_and_landing_timeout():
    landing, _, _, _ = setup()
    landing.request_land(1.0)
    landing.observe(feedback(mode_acknowledged=True), 1.1)
    landing.tick(2.0)
    assert landing.state == LandingState.LANDING_FAILED
    landing, _, _, _ = setup()
    landing.request_land(1.0)
    landing.observe(feedback(91.1, 2, mode_acknowledged=True), 91.1)
    landing.tick(91.1)
    assert landing.state == LandingState.LANDING_FAILED


@pytest.mark.parametrize("sample,now", [(feedback(stamp=float("nan")), 1.1),
                                      (feedback(stamp=1.2), 1.1),
                                      (feedback(stamp=1.1), 2.0)])
def test_invalid_timestamps(sample, now):
    landing, _, _, _ = setup()
    landing.request_land(1.0)
    landing.observe(sample, now)
    assert landing.state == LandingState.LANDING_FAILED


def test_replayed_telemetry_cannot_confirm_ground():
    landing, _, _, _ = setup()
    landing.request_land(1.0)
    landing.observe(feedback(mode_acknowledged=True), 1.1)
    landing.observe(feedback(1.2, 1, landed=True, armed=False), 1.2)
    assert landing.state == LandingState.LANDING_FAILED
    assert not landing.shutdown_ready(1.2)


def test_disarmed_in_air_is_not_landed_or_safe_shutdown():
    landing, controller, _, _ = setup()
    landing.request_land(1.0)
    landing.observe(feedback(mode_acknowledged=True, armed=False), 1.1)
    assert landing.state == LandingState.LANDING
    assert not landing.request_disarm(1.1) and not landing.shutdown_ready(1.1)
    assert controller.disarm_calls == 0


def test_disarm_rejection_timeout_and_rearm_block_shutdown():
    for failure in ("reject", "timeout", "rearm"):
        landing, controller, _, _ = setup()
        landing.request_land(1.0)
        landing.observe(feedback(mode_acknowledged=True, ground_contact_verified=True), 1.1)
        if failure == "reject":
            controller.disarm_accept = False
            assert not landing.request_disarm(1.1)
        elif failure == "timeout":
            assert landing.request_disarm(1.1)
            landing.observe(feedback(4.2, 2, landed=True), 4.2)
            landing.tick(4.2)
        else:
            landing.observe(feedback(1.2, 2, landed=True, armed=False), 1.2)
            assert landing.shutdown_ready(1.2)
            landing.observe(feedback(1.3, 3, landed=True, armed=True), 1.3)
        assert landing.state == LandingState.LANDING_FAILED
        assert not landing.shutdown_ready(4.2)


def test_simulated_controller_descent_requires_authoritative_ground_and_disarm():
    # Tiny controller-side kinematic mock, not the navigation double integrator
    # and not a model of this unknown aircraft or of any real FC firmware.
    landing, controller, _, _ = setup()
    assert landing.request_land(1.0)
    height = 1.0
    for step in range(1, 32):
        now = 1.0 + step * .1
        height = max(0., height - .05)
        grounded = height <= 0.
        landing.observe(feedback(now, step, mode_acknowledged=True, landed=grounded), now)
        landing.tick(now)
        if grounded: break
        assert landing.state == LandingState.LANDING and not landing.request_disarm(now)
    assert landing.state == LandingState.LANDED
    assert landing.request_disarm(now)
    landing.observe(feedback(now + .1, step + 1, landed=True, armed=False), now + .1)
    assert landing.shutdown_ready(now + .1) and controller.disarm_calls == 1


def test_readiness_never_certifies_unverified_hardware():
    report = evaluate(operator_authorized=True, telemetry_fresh=True, position_velocity_fresh=True)
    assert report["status"] == "NOT READY" and not report["flight_ready"]
    results = {check["condition"]: check for check in report["checks"]}
    for condition in ("controlled_landing", "controller_failsafe", "emergency_stop_manual_recovery",
                      "command_transformation", "measurement_covariance"):
        assert results[condition]["result"] == "UNVERIFIED"
    assert all(c["corrective_action"] for c in report["checks"])
