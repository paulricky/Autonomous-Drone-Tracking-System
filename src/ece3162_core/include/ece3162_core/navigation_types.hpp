#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace ece3162::navigation {

using Vec3 = std::array<double, 3>;

inline Vec3 add(const Vec3& a, const Vec3& b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
inline Vec3 sub(const Vec3& a, const Vec3& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline Vec3 mul(const Vec3& a, double s) {
  return {a[0] * s, a[1] * s, a[2] * s};
}
inline double dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline double norm(const Vec3& a) {
  return std::sqrt(dot(a, a));
}
inline bool finite(const Vec3& a) {
  return std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]);
}

struct PlanningState {
  Vec3 position{}, velocity{};
  double stamp{};
};
struct MotionPrimitive {
  Vec3 acceleration{};
  double duration{};
};
struct TrajectorySample {
  double time{};
  Vec3 position{}, velocity{}, acceleration{}, jerk{};
};
struct DynamicLimits {
  double max_velocity{1.5}, max_acceleration{2.0};
};

enum class FailureCode : std::uint8_t {
  kNone,
  kInvalidInput,
  kStartCollision,
  kGoalCollision,
  kInitialVelocity,
  kTimeout,
  kExpansionLimit,
  kUnreachable,
  kOptimizerFailure,
  kUnsafeTrajectory
};

struct PlannerResult {
  bool success{false};
  FailureCode code{FailureCode::kInvalidInput};
  std::string message;
  std::vector<PlanningState> states;
  std::vector<Vec3> accelerations;
  double cost{}, planning_time_s{};
  std::size_t expanded{}, generated{}, collision_checks{};
};

}  // namespace ece3162::navigation
