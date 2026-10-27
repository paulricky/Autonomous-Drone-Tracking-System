#pragma once
#include <deque>
#include <random>

#include "ece3162_core/navigation_types.hpp"

namespace ece3162::navigation {
struct DynamicsConfig {
  double dt{0.02}, max_velocity{2.0}, max_acceleration{2.0}, position_noise_std{},
      velocity_noise_std{}, feedback_delay{};
  unsigned seed{};
};
class DoubleIntegrator {
 public:
  DoubleIntegrator(PlanningState initial, DynamicsConfig config = {});
  const PlanningState& step(Vec3 acceleration);
  PlanningState estimated_state(bool valid = true);
  const PlanningState& ground_truth() const noexcept {
    return state_;
  }

 private:
  DynamicsConfig config_;
  PlanningState state_;
  std::deque<PlanningState> history_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0, 1};
};
}  // namespace ece3162::navigation
