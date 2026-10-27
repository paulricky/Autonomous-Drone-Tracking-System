#pragma once
#include <vector>

#include "ece3162_core/navigation_types.hpp"
#include "ece3162_core/voxel_map.hpp"

namespace ece3162::navigation {
struct FeasibilityReport {
  bool valid{}, collision_free{};
  double max_velocity{}, max_acceleration{}, minimum_clearance{};
  std::size_t velocity_violations{}, acceleration_violations{}, collision_checks{};
};
class BSpline {
 public:
  BSpline(std::vector<Vec3> control_points, double duration, int degree = 3);
  Vec3 position(double t) const;
  Vec3 velocity(double t) const;
  Vec3 acceleration(double t) const;
  Vec3 jerk(double t) const;
  TrajectorySample evaluate(double t) const;
  std::vector<TrajectorySample> sample(double dt) const;
  FeasibilityReport validate(const VoxelMap&, const DynamicLimits&, double sample_dt,
                             bool unknown_is_occupied = true) const;
  double duration() const noexcept {
    return duration_;
  }
  int degree() const noexcept {
    return degree_;
  }
  const std::vector<Vec3>& control_points() const noexcept {
    return points_;
  }
  const std::vector<double>& knots() const noexcept {
    return knots_;
  }

 private:
  static Vec3 evaluate_curve(const std::vector<Vec3>&, const std::vector<double>&, int, double);
  Vec3 derivative(double t, int order) const;
  std::vector<Vec3> points_;
  std::vector<double> knots_;
  double duration_;
  int degree_;
};
}  // namespace ece3162::navigation
