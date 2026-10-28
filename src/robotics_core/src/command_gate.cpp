#include "robotics_core/command_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace robotics::control {
CommandGate::CommandGate(GateLimits limits) : limits_(limits) {
  if (!std::isfinite(limits_.command_age_s) || limits_.command_age_s <= 0 ||
      !std::isfinite(limits_.acceleration_max_mps2) || limits_.acceleration_max_mps2 <= 0 ||
      !std::isfinite(limits_.telemetry_age_s) || limits_.telemetry_age_s <= 0 ||
      !std::isfinite(limits_.estimate_age_s) || limits_.estimate_age_s <= 0)
    throw std::invalid_argument("invalid command gate limits");
}
std::string CommandGate::check_common(const CommandSample& c, double now) const {
  if (!std::isfinite(now) || !std::isfinite(c.stamp) || c.stamp <= 0 || c.frame != "map")
    return "invalid command time or frame";
  const double age = now - c.stamp;
  if (age < -.05 || age > limits_.command_age_s) return "stale or future command";
  double norm_squared = 0;
  for (double value : c.acceleration) {
    if (!std::isfinite(value)) return "nonfinite acceleration";
    norm_squared += value * value;
  }
  if (std::sqrt(norm_squared) > limits_.acceleration_max_mps2 + 1e-6)
    return "acceleration exceeds limit";
  if (!c.braking && c.trajectory_id == 0) return "missing trajectory id";
  if (c.stamp <= last_stamp_ || (!c.braking && c.trajectory_id < last_trajectory_id_))
    return "replayed command";
  return {};
}
std::string CommandGate::check_physical(const CommandSample& c, double now,
                                        const PhysicalHealth& h) const {
  if (auto error = check_common(c, now); !error.empty()) return error;
  if (h.emergency_stop) return "emergency stop active";
  if (!h.operator_authorized) return "operator authorization absent";
  if (!h.controller_connected) return "flight controller disconnected";
  if (!h.protocol_verified || !h.configuration_verified)
    return "physical protocol or configuration unverified";
  if (!h.telemetry_valid || !std::isfinite(h.telemetry_stamp) ||
      now - h.telemetry_stamp < 0 || now - h.telemetry_stamp > limits_.telemetry_age_s)
    return "telemetry unavailable or stale";
  if (!h.estimate_healthy || !std::isfinite(h.estimate_stamp) ||
      now - h.estimate_stamp < 0 || now - h.estimate_stamp > limits_.estimate_age_s)
    return "state estimate unavailable or stale";
  if (!h.trajectory_valid || h.trajectory_id != c.trajectory_id ||
      !std::isfinite(h.trajectory_start) || !std::isfinite(h.trajectory_end) ||
      now < h.trajectory_start - .05 || now > h.trajectory_end)
    return "trajectory unavailable or stale";
  if (!h.operating_mode_known) return "operating mode unverified";
  return {};
}
void CommandGate::commit(const CommandSample& c) {
  last_stamp_ = c.stamp;
  if (c.trajectory_id > last_trajectory_id_) last_trajectory_id_ = c.trajectory_id;
}
}  // namespace robotics::control
