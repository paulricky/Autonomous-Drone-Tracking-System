"""Verify preflight rejects configuration before any transport starts."""
from pathlib import Path
import tempfile
import unittest

from launch import LaunchContext
import yaml

from robotics_bringup.hardware_config import validate


class HardwareConfigTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((Path(__file__).parents[1] / "config/hardware.yaml").read_text())

    def check_config(self, **options):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
            yaml.safe_dump(self.config, stream)
            stream.flush()
            context = LaunchContext()
            context.launch_configurations["config"] = stream.name
            return validate(context, **options)

    def test_unconfigured_cameras_rejected(self):
        with self.assertRaisesRegex(ValueError, "persistent device path"):
            self.check_config(cameras=True)

    def test_disabled_serial_rejected(self):
        with self.assertRaisesRegex(ValueError, "explicitly enabled"):
            self.check_config(telemetry=True)

    def test_integrated_bringup_requires_identity_and_closed_output(self):
        controller = self.config["msp_telemetry"]["ros__parameters"]
        with self.assertRaisesRegex(ValueError, "firmware identity"):
            self.check_config(integrated=True)
        controller.update(controller_board="bench inventory", firmware_family="bench inventory",
                          firmware_version="bench inventory")
        self.assertEqual(self.check_config(integrated=True), [])
        self.config["autonomy_pipeline"]["ros__parameters"]["auto_start"] = True
        with self.assertRaisesRegex(ValueError, "automatic mission disabled"):
            self.check_config(integrated=True)
