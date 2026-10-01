"""Report-quality 3-D planning visualization."""

from __future__ import annotations
from pathlib import Path as FilePath
import matplotlib.pyplot as plt
import numpy as np

from common.types import Path
from mapping.voxel_map import VoxelMap


def _voxel_scatter(axis, voxel_map: VoxelMap, *, color: str, alpha: float,
                   label: str, size: float) -> None:
    points = voxel_map.occupied_centers()
    if len(points):
        axis.scatter(points[:, 0], points[:, 1], points[:, 2], c=color,
                     alpha=alpha, s=size, marker="s", label=label)


def show_planning_result(raw_map: VoxelMap, inflated_map: VoxelMap,
                         start: np.ndarray, goal: np.ndarray, raw_path: Path,
                         smooth_path: Path, explored: np.ndarray | None = None,
                         *, title: str = "3-D Motion Planning", show: bool = True,
                         save_path: str | None = None) -> None:
    figure = plt.figure(figsize=(11, 8))
    axis = figure.add_subplot(111, projection="3d")
    _voxel_scatter(axis, inflated_map, color="orange", alpha=0.06,
                   label="inflated C-obstacles", size=18)
    _voxel_scatter(axis, raw_map, color="firebrick", alpha=0.35,
                   label="raw obstacles", size=24)
    if explored is not None and len(explored):
        # Keep rendering responsive without altering the planner's full metrics.
        stride = max(1, int(np.ceil(len(explored) / 6000)))
        displayed = explored[::stride]
        axis.scatter(displayed[:, 0], displayed[:, 1], displayed[:, 2],
                     c="lightsteelblue", alpha=0.18, s=5, label="A* explored")
    axis.scatter(*start, c="green", marker="o", s=100, label="START")
    axis.scatter(*goal, c="magenta", marker="*", s=150, label="GOAL")
    axis.plot(*raw_path.waypoints.T, color="royalblue", linewidth=1.5,
              linestyle="--", label="raw path")
    axis.plot(*smooth_path.waypoints.T, color="limegreen", linewidth=3,
              label="smoothed path")
    axis.set(xlabel="x / east [m]", ylabel="y / north [m]", zlabel="z / up [m]",
             xlim=(raw_map.minimum[0], raw_map.maximum[0]),
             ylim=(raw_map.minimum[1], raw_map.maximum[1]),
             zlim=(raw_map.minimum[2], raw_map.maximum[2]), title=title)
    axis.legend(loc="upper left", fontsize=8)
    figure.tight_layout()
    if save_path:
        output = FilePath(save_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output, dpi=180)
    if show:
        plt.show()
    else:
        plt.close(figure)
