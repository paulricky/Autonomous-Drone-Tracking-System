#pragma once
#include <array>
#include <optional>
#include <string>

namespace ece3162::control {
using Vector3 = std::array<double, 3>;
struct TrackerConfig { double kp{4.0}, kd{2.8}, max_acceleration{2.0}, max_error{2.0}, state_timeout{0.25}; };
struct Reference { Vector3 position{}, velocity{}, acceleration{}; double stamp{}; };
struct State { Vector3 position{}, velocity{}; double stamp{}; bool valid{false}; };
struct Command { Vector3 acceleration{}; double tracking_error{}; bool saturated{false}; };
class TrajectoryTracker {
 public:
  explicit TrajectoryTracker(TrackerConfig config = {});
  std::optional<Command> update(double now, const Reference& reference, const State& state);
  const std::string& failure_reason() const noexcept { return failure_reason_; }
 private: TrackerConfig config_; std::string failure_reason_;
};
}  // namespace ece3162::control
