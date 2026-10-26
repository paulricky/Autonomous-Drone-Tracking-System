"""Verify that the installed launch description can be constructed."""
import importlib.util
from pathlib import Path
import unittest

class LaunchDescriptionTest(unittest.TestCase):
    def test_diagnostics_launch_constructs(self):
        launch_file = Path(__file__).parents[1] / "launch" / "diagnostics.launch.py"
        spec = importlib.util.spec_from_file_location("diagnostics_launch", launch_file)
        module = importlib.util.module_from_spec(spec)
        self.assertIsNotNone(spec.loader)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        self.assertEqual(len(description.entities), 4)
