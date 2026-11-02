#pragma once
#include <array>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace robotics::flight {
using Json = nlohmann::json;
std::string authenticate(const std::string& key, const std::string& payload);
Json decode_operator_packet(const std::string& key, const std::string& packet, double wall_time);
std::string encode_operator_packet(const std::string& key, const Json& message);
std::string random_token();
struct OperatorDecision {
  std::string result;
  std::string operation;
  std::optional<std::array<double, 3>> direction;
};
// Monotonic time governs safety. Wall time is used only for authenticated packet expiration.
class OperatorSession {
 public:
  explicit OperatorSession(std::string frame);
  void observe(const Json& feedback, double now);
  void connected(double now);
  OperatorDecision handle(const Json& message, double now);
  OperatorDecision tick(double now);
  Json response(double now) const;
  bool navigation_permitted(double now) const;
  bool authorized() const {
    return enabled_ && !latched_;
  }
  bool latched() const {
    return latched_;
  }

 private:
  bool fresh(double now) const;
  bool flag(const char* key) const;
  OperatorDecision inhibit(const std::string& operation);
  std::string frame_, landing_state_{"idle"}, landing_detail_{"no landing requested"};
  Json feedback_ = Json::object();
  double feedback_time_{-1e9}, packet_time_{-1e9}, direction_time_{-1e9}, landing_start_{};
  bool enabled_{}, latched_{}, quit_{}, arm_sent_{}, takeoff_sent_{}, tracking_seen_{};
};
}  // namespace robotics::flight
