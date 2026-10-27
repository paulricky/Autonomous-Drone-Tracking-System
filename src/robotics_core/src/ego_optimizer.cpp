#include "robotics_core/ego_optimizer.hpp"

#include <chrono>
#include <stdexcept>

namespace robotics::navigation {
EgoOptimizer::EgoOptimizer(EgoConfig c) : config_(c) {
  if (c.degree < 1 || c.clearance <= 0 || c.max_iterations == 0 || c.initial_step <= 0 ||
      c.sample_dt <= 0)
    throw std::invalid_argument("invalid EGO optimizer config");
}
ObjectiveBreakdown EgoOptimizer::objective(const std::vector<Vec3>& p,
                                           const std::vector<Vec3>& guide, const VoxelMap& map,
                                           double duration, const DynamicLimits& limits,
                                           std::vector<Vec3>* grad) const {
  if (p.size() != guide.size() || p.size() < static_cast<std::size_t>(config_.degree + 1) ||
      duration <= 0)
    throw std::invalid_argument("invalid optimizer input");
  if (grad)
    grad->assign(p.size(), {});
  ObjectiveBreakdown c;
  const auto obs = map.occupied_centers();
  const auto knots = BSpline(p, duration, config_.degree).knots();
  for (std::size_t i = 1; i + 1 < p.size(); ++i) {
    const Vec3 second = add(sub(p[i - 1], mul(p[i], 2)), p[i + 1]);
    c.smoothness += config_.smoothness_weight * dot(second, second);
    if (grad) {
      const auto g = mul(second, 2 * config_.smoothness_weight);
      (*grad)[i - 1] = add((*grad)[i - 1], g);
      (*grad)[i] = sub((*grad)[i], mul(g, 2));
      (*grad)[i + 1] = add((*grad)[i + 1], g);
    }
  }
  for (std::size_t i = 1; i + 1 < p.size(); ++i) {
    const auto d = sub(p[i], guide[i]);
    c.guidance += config_.guidance_weight * dot(d, d);
    if (grad)
      (*grad)[i] = add((*grad)[i], mul(d, 2 * config_.guidance_weight));
    double best = 1e100;
    Vec3 delta{};
    for (const auto& o : obs) {
      auto q = sub(p[i], o);
      double n = norm(q);
      if (n < best) {
        best = n;
        delta = q;
      }
    }
    if (best < config_.clearance) {
      const double e = config_.clearance - best;
      c.collision += config_.collision_weight * e * e;
      if (grad && best > 1e-9)
        (*grad)[i] = add((*grad)[i], mul(delta, -2 * config_.collision_weight * e / best));
    }
  }
  std::vector<double> velocity_scale;
  velocity_scale.reserve(p.size() - 1);
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    const double span = knots[i + config_.degree + 1] - knots[i + 1];
    const double scale = span > 0 ? double(config_.degree) / span : 0.0;
    velocity_scale.push_back(scale);
    Vec3 dv = mul(sub(p[i + 1], p[i]), scale);
    const double speed = norm(dv);
    if (speed > limits.max_velocity) {
      const double e = speed - limits.max_velocity;
      c.feasibility += config_.feasibility_weight * e * e;
      if (grad && speed > 1e-12) {
        auto g = mul(dv, 2 * config_.feasibility_weight * e * scale / speed);
        (*grad)[i] = sub((*grad)[i], g);
        (*grad)[i + 1] = add((*grad)[i + 1], g);
      }
    }
  }
  if (config_.degree >= 2) {
    for (std::size_t i = 0; i + 2 < p.size(); ++i) {
      const double span = knots[i + config_.degree + 1] - knots[i + 2];
      if (span <= 0)
        continue;
      const double scale = double(config_.degree - 1) / span;
      const double left = scale * velocity_scale[i];
      const double right = scale * velocity_scale[i + 1];
      const Vec3 acceleration =
          add(mul(p[i], left), add(mul(p[i + 1], -left - right), mul(p[i + 2], right)));
      const double magnitude = norm(acceleration);
      if (magnitude <= limits.max_acceleration)
        continue;
      const double excess = magnitude - limits.max_acceleration;
      c.feasibility += config_.feasibility_weight * excess * excess;
      if (grad && magnitude > 1e-12) {
        const auto g = mul(acceleration, 2 * config_.feasibility_weight * excess / magnitude);
        (*grad)[i] = add((*grad)[i], mul(g, left));
        (*grad)[i + 1] = sub((*grad)[i + 1], mul(g, left + right));
        (*grad)[i + 2] = add((*grad)[i + 2], mul(g, right));
      }
    }
  }
  if (grad) {
    (*grad).front() = {};
    (*grad).back() = {};
  }
  return c;
}
EgoResult EgoOptimizer::optimize(const std::vector<Vec3>& guide, const VoxelMap& map,
                                 double duration, const DynamicLimits& limits) const {
  const auto begun = std::chrono::steady_clock::now();
  EgoResult r;
  if (guide.size() < static_cast<std::size_t>(config_.degree + 1)) {
    r.message = "insufficient guidance points";
    return r;
  }
  for (const auto& p : guide)
    if (!finite(p)) {
      r.message = "non-finite guidance";
      return r;
    }
  auto points = guide;
  std::vector<Vec3> gradient;
  r.initial = objective(points, guide, map, duration, limits, &gradient);
  ++r.objective_evaluations;
  double value = r.initial.total();
  for (std::size_t iter = 0; iter < config_.max_iterations; ++iter) {
    r.iterations = iter + 1;
    objective(points, guide, map, duration, limits, &gradient);
    ++r.objective_evaluations;
    double gn = 0;
    for (std::size_t i = 1; i + 1 < gradient.size(); ++i)
      gn += dot(gradient[i], gradient[i]);
    gn = std::sqrt(gn);
    if (gn < config_.gradient_tolerance)
      break;
    double step = config_.initial_step;
    bool accepted = false;
    for (int ls = 0; ls < 20; ++ls) {
      auto candidate = points;
      for (std::size_t i = 1; i + 1 < candidate.size(); ++i)
        candidate[i] = sub(candidate[i], mul(gradient[i], step));
      auto cost = objective(candidate, guide, map, duration, limits, nullptr);
      ++r.objective_evaluations;
      if (cost.total() <= value - 1e-4 * step * gn * gn) {
        points = std::move(candidate);
        value = cost.total();
        accepted = true;
        break;
      }
      step *= .5;
    }
    if (!accepted)
      break;
  }
  r.final = objective(points, guide, map, duration, limits, nullptr);
  ++r.objective_evaluations;
  r.trajectory = std::make_shared<BSpline>(points, duration, config_.degree);
  r.feasibility = r.trajectory->validate(map, limits, config_.sample_dt);
  r.success = r.feasibility.valid && std::isfinite(r.final.total());
  r.code = r.success ? FailureCode::kNone : FailureCode::kUnsafeTrajectory;
  r.message = r.success ? "optimized trajectory valid"
                        : "optimized trajectory rejected by curve validation";
  if (!r.success)
    r.trajectory.reset();
  r.optimization_time_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begun).count();
  return r;
}
}  // namespace robotics::navigation
