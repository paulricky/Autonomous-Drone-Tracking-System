"""ESDF-free, EGO-Planner-inspired local B-spline optimization.

This native optimizer follows EGO's broad control-point optimization strategy
(smoothness, collision, and dynamic feasibility costs) but is not a port of the
reference ROS/C++ rebound optimizer and does not claim algorithmic equivalence.
"""

from __future__ import annotations
from dataclasses import dataclass
import time
import numpy as np
from scipy.optimize import minimize

from ece3162.mapping.voxel_map import VoxelMap
from ece3162.planning.collision_checker import CollisionChecker
from ece3162.trajectory.bspline import BSplineTrajectory, FeasibilityReport


@dataclass(frozen=True, slots=True)
class EgoConfig:
    degree: int = 3
    clearance: float = 0.5
    smoothness_weight: float = 1.0
    collision_weight: float = 100.0
    guidance_weight: float = 2.0
    max_iterations: int = 120
    sample_dt: float = 0.03


@dataclass(slots=True)
class EgoResult:
    success: bool
    trajectory: BSplineTrajectory | None
    optimization_time: float
    initial_cost: float
    final_cost: float
    feasibility: FeasibilityReport | None
    message: str


class EgoInspiredOptimizer:
    def __init__(self, config: EgoConfig = EgoConfig()) -> None: self.config = config

    def optimize(self, guidance: np.ndarray, voxel_map: VoxelMap, duration: float,
                 max_velocity: float, max_acceleration: float) -> EgoResult:
        begin = time.perf_counter(); guide = np.asarray(guidance, dtype=float)
        if guide.ndim != 2 or guide.shape[1] != 3 or len(guide) < self.config.degree + 1:
            raise ValueError("guidance must contain at least degree+1 finite 3-D points")
        occupied = voxel_map.occupied_centers()
        fixed_first, fixed_last = guide[0].copy(), guide[-1].copy()
        def objective(flat: np.ndarray) -> float:
            cp = np.vstack((fixed_first, flat.reshape(-1, 3), fixed_last))
            second = cp[:-2] - 2*cp[1:-1] + cp[2:]
            cost = self.config.smoothness_weight * float(np.sum(second**2))
            cost += self.config.guidance_weight * float(np.sum((cp-guide)**2))
            if len(occupied):
                for point in cp[1:-1]:
                    delta = point - occupied
                    distance = np.linalg.norm(delta, axis=1).min()
                    cost += self.config.collision_weight * max(0.0, self.config.clearance-distance)**2
            return cost
        x0 = guide[1:-1].ravel(); initial = objective(x0)
        solution = minimize(objective, x0, method="L-BFGS-B", options={"maxiter": self.config.max_iterations})
        cp = np.vstack((fixed_first, solution.x.reshape(-1, 3), fixed_last))
        spline = BSplineTrajectory(cp, duration, self.config.degree)
        feasibility = spline.validate(CollisionChecker(voxel_map), max_velocity, max_acceleration, self.config.sample_dt)
        # An iteration-limit status is acceptable only when the independently
        # sampled trajectory validator proves the best candidate safe/feasible.
        success = bool(feasibility.valid and np.isfinite(solution.fun))
        message = "optimized trajectory valid" if success else f"rejected: optimizer={solution.message}; feasible={feasibility.valid}"
        return EgoResult(success, spline if success else None, time.perf_counter()-begin,
                         initial, float(solution.fun), feasibility, message)
