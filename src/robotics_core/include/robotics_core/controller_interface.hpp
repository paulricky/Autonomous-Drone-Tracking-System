#pragma once
#include <array>
#include <string>

namespace robotics::flight {
enum class ControllerOperation { Arm, Takeoff, Hold, Land, Emergency, Disarm };
struct ControllerCapabilities {
  bool identity_verified{}, protocol_verified{}, arm{}, takeoff{}, acceleration{}, hold{}, land{},
      emergency{}, ground_feedback{};
};
struct ControllerFeedback {
  bool connected{}, armed{}, on_ground{}, airborne{}, manual_override{};
  std::string mode, phase, detail;
};
struct MotionSetpoint {
  double stamp{};
  std::array<double, 3> acceleration_enu{};  // Net m/s², excludes gravity, never thrust or raw RC.
  bool braking{};
  std::string frame;
};
// Request submission is distinct from controller acknowledgement and observed execution.
class ControllerBackend {
 public:
  virtual ~ControllerBackend() = default;
  virtual ControllerCapabilities capabilities() const = 0;
  virtual ControllerFeedback feedback() const = 0;
  virtual void request(ControllerOperation operation) = 0;
  virtual bool submit_motion(const MotionSetpoint& setpoint) = 0;
};
// The recovered MSP v1 protocol establishes telemetry/RC transport only. No safe
// autonomous takeoff/hold/landing or authoritative landed feedback was established.
class UnverifiedMspBackend final : public ControllerBackend {
 public:
  ControllerCapabilities capabilities() const override {
    return {};
  }
  ControllerFeedback feedback() const override {
    return {false,     false,
            false,     false,
            false,     "unknown",
            "blocked", "MSP raw RC lacks verified stabilized autonomy and landing/ground feedback"};
  }
  void request(ControllerOperation) override {
  }
  bool submit_motion(const MotionSetpoint&) override {
    return false;
  }
};
}  // namespace robotics::flight
