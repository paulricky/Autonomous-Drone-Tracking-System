#include "ece3162_core/bspline.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace ece3162::navigation {
BSpline::BSpline(std::vector<Vec3> p, double duration, int degree)
    : points_(std::move(p)), duration_(duration), degree_(degree) {
  if (degree < 1 || !std::isfinite(duration) || duration <= 0 ||
      points_.size() < static_cast<std::size_t>(degree + 1))
    throw std::invalid_argument("invalid B-spline dimensions");
  for (const auto& v : points_)
    if (!finite(v))
      throw std::invalid_argument("non-finite control point");
  const auto n = points_.size();
  knots_.assign(n + degree + 1, 0);
  for (int i = 0; i <= degree; ++i)
    knots_[n + i] = duration;
  const std::size_t interiors = n - degree - 1;
  for (std::size_t i = 1; i <= interiors; ++i)
    knots_[degree + i] = duration * double(i) / double(interiors + 1);
}
Vec3 BSpline::evaluate_curve(const std::vector<Vec3>& p, const std::vector<double>& k, int degree,
                             double t) {
  if (p.empty())
    return {};
  if (degree == 0) {
    auto it = std::upper_bound(k.begin(), k.end(), t);
    std::size_t i = it == k.begin() ? 0 : static_cast<std::size_t>(it - k.begin() - 1);
    return p[std::min(i, p.size() - 1)];
  }
  const std::size_t n = p.size() - 1;
  std::size_t span = n;
  if (t < k[n + 1]) {
    auto it = std::upper_bound(k.begin() + degree, k.begin() + n + 2, t);
    span = static_cast<std::size_t>(it - k.begin() - 1);
  }
  std::vector<Vec3> d(degree + 1);
  for (int j = 0; j <= degree; ++j)
    d[j] = p[span - degree + j];
  for (int r = 1; r <= degree; ++r)
    for (int j = degree; j >= r; --j) {
      const std::size_t i = span - degree + j;
      const double den = k[i + degree - r + 1] - k[i];
      const double a = den > 0 ? (t - k[i]) / den : 0;
      d[j] = add(mul(d[j - 1], 1 - a), mul(d[j], a));
    }
  return d[degree];
}
Vec3 BSpline::derivative(double t, int order) const {
  if (!std::isfinite(t) || t < 0 || t > duration_)
    throw std::out_of_range("B-spline time outside duration");
  if (order > degree_)
    return {};
  auto p = points_;
  auto k = knots_;
  int d = degree_;
  for (int r = 0; r < order; ++r) {
    std::vector<Vec3> q;
    q.reserve(p.size() - 1);
    for (std::size_t i = 0; i + 1 < p.size(); ++i) {
      const double den = k[i + d + 1] - k[i + 1];
      q.push_back(den > 0 ? mul(sub(p[i + 1], p[i]), double(d) / den) : Vec3{});
    }
    p = std::move(q);
    k.erase(k.begin());
    k.pop_back();
    --d;
  }
  return evaluate_curve(p, k, d, t);
}
Vec3 BSpline::position(double t) const {
  if (!std::isfinite(t) || t < 0 || t > duration_)
    throw std::out_of_range("B-spline time outside duration");
  return evaluate_curve(points_, knots_, degree_, t);
}
Vec3 BSpline::velocity(double t) const {
  return derivative(t, 1);
}
Vec3 BSpline::acceleration(double t) const {
  return derivative(t, 2);
}
Vec3 BSpline::jerk(double t) const {
  return derivative(t, 3);
}
TrajectorySample BSpline::evaluate(double t) const {
  return {t, position(t), velocity(t), acceleration(t), jerk(t)};
}
std::vector<TrajectorySample> BSpline::sample(double dt) const {
  if (!std::isfinite(dt) || dt <= 0)
    throw std::invalid_argument("sample dt must be positive");
  const auto n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(duration_ / dt)));
  std::vector<TrajectorySample> out;
  out.reserve(n + 1);
  for (std::size_t i = 0; i <= n; ++i)
    out.push_back(evaluate(duration_ * double(i) / n));
  return out;
}
FeasibilityReport BSpline::validate(const VoxelMap& m, const DynamicLimits& lim, double dt,
                                    bool unknown) const {
  if (lim.max_velocity <= 0 || lim.max_acceleration <= 0)
    throw std::invalid_argument("invalid dynamic limits");
  FeasibilityReport r{};
  r.collision_free = true;
  r.minimum_clearance = std::numeric_limits<double>::infinity();
  auto occupied = m.occupied_centers();
  auto samples = sample(dt);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const double v = norm(samples[i].velocity), a = norm(samples[i].acceleration);
    r.max_velocity = std::max(r.max_velocity, v);
    r.max_acceleration = std::max(r.max_acceleration, a);
    r.velocity_violations += v > lim.max_velocity + 1e-9;
    r.acceleration_violations += a > lim.max_acceleration + 1e-9;
    if (m.occupied(samples[i].position, unknown))
      r.collision_free = false;
    for (const auto& o : occupied)
      r.minimum_clearance = std::min(r.minimum_clearance, norm(sub(samples[i].position, o)));
    if (i && m.segment_occupied(samples[i - 1].position, samples[i].position, m.resolution() * .2,
                                unknown, &r.collision_checks))
      r.collision_free = false;
  }
  r.valid = r.collision_free && !r.velocity_violations && !r.acceleration_violations;
  return r;
}
}  // namespace ece3162::navigation
