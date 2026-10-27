#pragma once
#include "ece3162_core/navigation_types.hpp"
#include "ece3162_core/voxel_map.hpp"

namespace ece3162::navigation {
struct KinodynamicConfig {
  double primitive_duration{0.5}, max_velocity{1.5}, max_acceleration{1.0};
  double velocity_resolution{0.5}, position_resolution{0.0};
  double goal_position_tolerance{0.35}, goal_velocity_tolerance{0.55};
  double collision_sample_dt{0.05}, heuristic_weight{1.0}, timeout_s{10.0};
  std::size_t max_expansions{100000};
  bool diagonal_accelerations{false};
};
class KinodynamicAStar {
 public:
  explicit KinodynamicAStar(KinodynamicConfig config = {});
  static PlanningState propagate(const PlanningState&, const MotionPrimitive&);
  PlannerResult plan(const PlanningState& start, const Vec3& goal, const VoxelMap& map,
                     Vec3 terminal_velocity = {}) const;

 private:
  KinodynamicConfig config_;
};
}  // namespace ece3162::navigation
