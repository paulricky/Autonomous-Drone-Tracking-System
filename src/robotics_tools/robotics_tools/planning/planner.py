"""Common planner interface."""

from __future__ import annotations
from abc import ABC, abstractmethod
from numpy.typing import ArrayLike
from robotics_tools.common.types import Path, PlannerResult
from robotics_tools.mapping.voxel_map import VoxelMap


class MotionPlanner(ABC):
    @abstractmethod
    def plan(self, start: ArrayLike, goal: ArrayLike, voxel_map: VoxelMap) -> PlannerResult:
        raise NotImplementedError

    @abstractmethod
    def is_path_valid(self, path: Path, voxel_map: VoxelMap) -> bool:
        raise NotImplementedError

    def reset(self) -> None:
        """Discard planner-specific search state."""

