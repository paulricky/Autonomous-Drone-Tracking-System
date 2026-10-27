"""Colcon-level checks for the installed algorithm package."""
import numpy as np
import unittest

from robotics_tools.mapping.voxel_map import Occupancy, VoxelMap


class InstalledPackageTest(unittest.TestCase):
    def test_voxel_map_retains_unknown_space(self):
        voxel_map = VoxelMap(np.zeros(3), np.ones(3), 0.25)
        index = voxel_map.world_to_grid(np.array([0.5, 0.5, 0.5]))
        self.assertEqual(Occupancy(voxel_map.data[index]), Occupancy.UNKNOWN)
