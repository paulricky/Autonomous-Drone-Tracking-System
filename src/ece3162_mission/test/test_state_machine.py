import unittest
from ece3162_mission.state_machine import MissionMachine, MissionState

class MissionMachineTest(unittest.TestCase):
    def test_successful_mission_transition(self):
        mission = MissionMachine()
        self.assertTrue(mission.request(True, True))
        mission.plan_ready(); mission.complete()
        self.assertIs(mission.state, MissionState.SUCCEEDED)

    def test_invalid_inputs_fail_closed(self):
        mission = MissionMachine()
        self.assertFalse(mission.request(False, True))
        self.assertIs(mission.state, MissionState.FAILED)

    def test_out_of_order_transition_rejected(self):
        with self.assertRaises(RuntimeError): MissionMachine().complete()
