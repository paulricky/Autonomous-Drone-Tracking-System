#include "robotics_core/trajectory_tracker.hpp"

#include <cmath>
#include <stdexcept>

namespace robotics::control {
TrajectoryTracker::TrajectoryTracker(TrackerConfig config) : config_(config) {
  if (config_.kp < 0 || config_.kd < 0 || config_.max_acceleration <= 0 || config_.max_error <= 0 ||
      config_.state_timeout <= 0)
    throw std::invalid_argument("invalid tracker configuration");
}
std::optional<Command> TrajectoryTracker::update(double now, const Reference& ref,
                                                 const State& state) {
  failure_reason_.clear();
  if (!state.valid || !std::isfinite(now) || now - state.stamp > config_.state_timeout ||
      now < state.stamp) {
    failure_reason_ = "invalid or stale localization";
    return std::nullopt;
  }
  Command out;
  double squared_error = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(state.position[i]) || !std::isfinite(state.velocity[i]) ||
        !std::isfinite(ref.position[i]) || !std::isfinite(ref.velocity[i]) ||
        !std::isfinite(ref.acceleration[i]) || !std::isfinite(ref.stamp)) {
      failure_reason_ = "non-finite state or reference";
      return std::nullopt;
    }
    const double error = ref.position[i] - state.position[i];
    squared_error += error * error;
    out.acceleration[i] = ref.acceleration[i] + config_.kp * error +
                          config_.kd * (ref.velocity[i] - state.velocity[i]);
  }
  out.tracking_error = std::sqrt(squared_error);
  if (out.tracking_error > config_.max_error) {
    failure_reason_ = "excessive tracking error";
    return std::nullopt;
  }
  double norm2 = 0.0;
  for (const auto value : out.acceleration)
    norm2 += value * value;
  const double norm = std::sqrt(norm2);
  if (norm > config_.max_acceleration) {
    out.saturated = true;
    for (auto& value : out.acceleration)
      value *= config_.max_acceleration / norm;
  }
  return out;
}
}  // namespace robotics::control
