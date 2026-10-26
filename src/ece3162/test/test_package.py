"""Colcon-level checks for the installed algorithm package."""
import numpy as np
import unittest

from ece3162.hardware.msp import encode_raw_rc
from ece3162.mapping.voxel_map import Occupancy, VoxelMap


class InstalledPackageTest(unittest.TestCase):
    def test_msp_encoder_is_available(self):
        packet = encode_raw_rc([1500, 1500, 1500, 1000])
        self.assertTrue(packet.startswith(b"$M<"))
        self.assertEqual(packet[4], 200)

    def test_voxel_map_retains_unknown_space(self):
        voxel_map = VoxelMap(np.zeros(3), np.ones(3), 0.25)
        index = voxel_map.world_to_grid(np.array([0.5, 0.5, 0.5]))
        self.assertEqual(Occupancy(voxel_map.data[index]), Occupancy.UNKNOWN)
