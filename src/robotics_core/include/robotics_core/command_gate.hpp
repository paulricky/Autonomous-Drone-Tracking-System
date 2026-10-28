#pragma once
#include <array>
#include <cstdint>
#include <string>

namespace robotics::control {
struct CommandSample {
  double stamp{};
  std::uint64_t trajectory_id{};
  std::array<double, 3> acceleration{};
  bool braking{};
  std::string frame;
};
struct PhysicalHealth {
  bool operator_authorized{}, controller_connected{}, protocol_verified{};
  bool telemetry_valid{}, estimate_healthy{}, trajectory_valid{};
  bool operating_mode_known{}, emergency_stop{}, configuration_verified{};
  double telemetry_stamp{}, estimate_stamp{}, trajectory_start{}, trajectory_end{};
  std::uint64_t trajectory_id{};
};
struct GateLimits {
  double command_age_s{.15}, acceleration_max_mps2{2.0};
  double telemetry_age_s{.25}, estimate_age_s{.25};
};
class CommandGate {
 public:
  explicit CommandGate(GateLimits limits = {});
  // Empty string means accepted. The caller may commit only an accepted sample.
  std::string check_common(const CommandSample&, double now) const;
  std::string check_physical(const CommandSample&, double now, const PhysicalHealth&) const;
  void commit(const CommandSample&);
 private:
  GateLimits limits_;
  double last_stamp_{};
  std::uint64_t last_trajectory_id_{};
};
}  // namespace robotics::control
