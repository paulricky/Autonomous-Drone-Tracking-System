#include "ece3162_core/dynamics.hpp"

#include <stdexcept>
namespace ece3162::navigation {
DoubleIntegrator::DoubleIntegrator(PlanningState s, DynamicsConfig c)
    : config_(c), state_(s), rng_(c.seed) {
  if (!finite(s.position) || !finite(s.velocity) || c.dt <= 0 || c.max_velocity <= 0 ||
      c.max_acceleration <= 0 || c.feedback_delay < 0)
    throw std::invalid_argument("invalid dynamics configuration");
  history_.push_back(s);
}
const PlanningState& DoubleIntegrator::step(Vec3 a) {
  if (!finite(a))
    throw std::invalid_argument("non-finite command");
  double n = norm(a);
  if (n > config_.max_acceleration)
    a = mul(a, config_.max_acceleration / n);
  state_.position = add(add(state_.position, mul(state_.velocity, config_.dt)),
                        mul(a, .5 * config_.dt * config_.dt));
  state_.velocity = add(state_.velocity, mul(a, config_.dt));
  n = norm(state_.velocity);
  if (n > config_.max_velocity)
    state_.velocity = mul(state_.velocity, config_.max_velocity / n);
  state_.stamp += config_.dt;
  history_.push_back(state_);
  while (history_.size() > 1 && history_[1].stamp <= state_.stamp - config_.feedback_delay)
    history_.pop_front();
  return state_;
}
PlanningState DoubleIntegrator::estimated_state(bool valid) {
  auto s = history_.front();
  for (int i = 0; i < 3; ++i) {
    s.position[i] += config_.position_noise_std * normal_(rng_);
    s.velocity[i] += config_.velocity_noise_std * normal_(rng_);
  }
  if (!valid)
    s.stamp = -1;
  return s;
}
}  // namespace ece3162::navigation
