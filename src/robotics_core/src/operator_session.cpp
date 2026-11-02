#include "robotics_core/operator_session.hpp"

#include <openssl/crypto.h>
#include <openssl/hmac.h>
#include <openssl/rand.h>

#include <cmath>
#include <stdexcept>

namespace robotics::flight {
static std::string hex(const unsigned char* data, size_t size) {
  static const char digits[] = "0123456789abcdef";
  std::string result;
  result.reserve(size * 2);
  for (size_t i = 0; i < size; ++i) {
    result += digits[data[i] >> 4];
    result += digits[data[i] & 15];
  }
  return result;
}
std::string random_token() {
  unsigned char bytes[32];
  if (RAND_bytes(bytes, sizeof(bytes)) != 1)
    throw std::runtime_error("secure session entropy unavailable");
  return hex(bytes, sizeof(bytes));
}
std::string authenticate(const std::string& key, const std::string& payload) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned size = 0;
  if (key.size() < 32 ||
      !HMAC(EVP_sha256(), key.data(), int(key.size()),
            reinterpret_cast<const unsigned char*>(payload.data()), payload.size(), digest, &size))
    throw std::runtime_error("HMAC unavailable or key too short");
  return hex(digest, size);
}
Json decode_operator_packet(const std::string& key, const std::string& packet, double wall) {
  if (packet.size() > 8192 || packet.size() < 67 || packet[64] != '\n')
    throw std::invalid_argument("packet size/signature");
  auto payload = packet.substr(65), signature = authenticate(key, payload);
  if (CRYPTO_memcmp(signature.data(), packet.data(), 64) != 0)
    throw std::invalid_argument("authentication failed");
  auto message = Json::parse(payload);
  if (!message.is_object() || !message.contains("time") || !message["time"].is_number())
    throw std::invalid_argument("invalid packet timestamp");
  double stamp = message["time"].get<double>();
  if (!std::isfinite(stamp) || !std::isfinite(wall) || std::abs(wall - stamp) > .4)
    throw std::invalid_argument("expired packet or clock skew");
  return message;
}
std::string encode_operator_packet(const std::string& key, const Json& message) {
  auto payload = message.dump(-1, ' ', true);
  return authenticate(key, payload) + "\n" + payload;
}
OperatorSession::OperatorSession(std::string frame) : frame_(std::move(frame)) {
  if (frame_.empty() || frame_ == "UNCONFIGURED")
    throw std::invalid_argument("measured pointing reference required");
}
bool OperatorSession::flag(const char* key) const {
  auto it = feedback_.find(key);
  return it != feedback_.end() && it->is_boolean() && it->get<bool>();
}
bool OperatorSession::fresh(double now) const {
  return now >= feedback_time_ && now - feedback_time_ < .4 && flag("connected");
}
void OperatorSession::observe(const Json& feedback, double now) {
  if (!feedback.is_object() || !std::isfinite(now))
    return;
  feedback_ = feedback;
  feedback_time_ = now;
  if (landing_state_ == "idle" || landing_state_ == "landing_failed")
    return;
  if (!flag("connected") || !flag("identity_verified")) {
    landing_state_ = "landing_failed";
    landing_detail_ = "authoritative controller feedback lost";
    return;
  }
  if (flag("ground")) {
    if (feedback_.contains("armed") && feedback_["armed"].is_boolean()) {
      landing_state_ = flag("armed") ? "landed" : "disarmed";
      landing_detail_ = "controller ground feedback confirmed";
    }
  } else if (landing_state_ == "landed" || landing_state_ == "disarmed") {
    landing_state_ = "landing_failed";
    landing_detail_ = "ground confirmation lost";
  } else if (feedback_.value("mode", std::string{}) == "AUTO.LAND") {
    landing_state_ = "landing";
    landing_detail_ = "controller landing mode confirmed";
  }
}
void OperatorSession::connected(double now) {
  enabled_ = false;
  tracking_seen_ = false;
  direction_time_ = -1e9;
  packet_time_ = now;
  // Reconnection never clears landing, emergency, arming or takeoff latches.
}
OperatorDecision OperatorSession::inhibit(const std::string& operation) {
  enabled_ = false;
  tracking_seen_ = false;
  direction_time_ = -1e9;
  if (operation == "LAND" || operation == "EMERGENCY") {
    latched_ = true;
    if (landing_state_ == "idle") {
      landing_state_ = "land_requested";
      landing_detail_ = "awaiting controller feedback";
    }
  }
  return {operation + " requested; controller feedback pending", operation, std::nullopt};
}
OperatorDecision OperatorSession::handle(const Json& m, double now) {
  packet_time_ = now;
  auto command = m.value("command", std::string{});
  if (command == "EMERGENCY" || command == "LAND" || command == "HOLD") {
    if (!latched_ && (command == "LAND" || command == "EMERGENCY"))
      landing_start_ = now;
    return inhibit(command);
  }
  if (command == "QUIT") {
    quit_ = true;
    if (!fresh(now) || !flag("identity_verified") || !flag("ground") || flag("armed")) {
      if (!latched_)
        landing_start_ = now;
      return inhibit("LAND");
    }
    return {"ground and disarming confirmed; safe to close", {}, std::nullopt};
  }
  if (command == "ENABLE") {
    if (latched_ || quit_)
      return {"REJECTED: landing/emergency latch; ground restart required", {}, std::nullopt};
    enabled_ = true;
    return {"operator enabled; motors unchanged", {}, std::nullopt};
  }
  if (command == "DISARM") {
    if (fresh(now) && flag("identity_verified") && flag("ground"))
      return {"disarm requested; awaiting controller feedback", "DISARM", std::nullopt};
    return {"REJECTED: authoritative ground not confirmed", {}, std::nullopt};
  }
  if (command == "ARM" || command == "TAKEOFF") {
    if (!fresh(now) || !enabled_ || latched_ || !flag("ready") || !flag("identity_verified") ||
        !flag("ground"))
      return {"REJECTED: authorization/readiness/ground interlock", {}, std::nullopt};
    if (command == "ARM" && !arm_sent_ && !flag("armed")) {
      arm_sent_ = true;
      return {"arm requested; awaiting controller feedback", "ARM", std::nullopt};
    }
    // A single T requests takeoff; the native controller independently checks mode and
    // authorization.
    if (command == "TAKEOFF" && arm_sent_ && flag("armed") && flag("takeoff_allowed") &&
        !takeoff_sent_) {
      takeoff_sent_ = true;
      return {"takeoff requested; awaiting controller flight feedback", "TAKEOFF", std::nullopt};
    }
    return {"REJECTED: invalid or repeated flight transition", {}, std::nullopt};
  }
  if (command == "DIRECTION") {
    if (!enabled_ || latched_ || !fresh(now) || !flag("ready") || !flag("airborne") ||
        feedback_.value("mode", std::string{}) == "AUTO.TAKEOFF" ||
        m.value("frame", std::string{}) != frame_ || !m.contains("operator_enabled") ||
        m["operator_enabled"] != true)
      return {"REJECTED direction: authorization, frame or flight state", {}, std::nullopt};
    if (!m.contains("direction") || !m["direction"].is_array() || m["direction"].size() != 3)
      return {"REJECTED invalid direction", {}, std::nullopt};
    std::array<double, 3> d{};
    double norm2 = 0;
    for (size_t i = 0; i < 3; ++i) {
      if (!m["direction"][i].is_number())
        return {"REJECTED invalid direction", {}, std::nullopt};
      d[i] = m["direction"][i].get<double>();
      norm2 += d[i] * d[i];
    }
    if (!std::isfinite(norm2) || std::abs(norm2 - 1) > .02)
      return {"REJECTED invalid direction", {}, std::nullopt};
    double norm = std::sqrt(norm2);
    for (auto& value : d)
      value /= norm;
    direction_time_ = now;
    tracking_seen_ = true;
    return {"intent accepted; onboard planner retains movement authority", {}, d};
  }
  if (command == "HEARTBEAT")
    return {"heartbeat", {}, std::nullopt};
  return {"REJECTED unsupported operator command", {}, std::nullopt};
}
OperatorDecision OperatorSession::tick(double now) {
  if (!latched_ && now - packet_time_ > .6 && (enabled_ || flag("armed"))) {
    landing_start_ = now;
    return inhibit("EMERGENCY");
  }
  if (latched_ && landing_state_ != "disarmed" && landing_state_ != "landing_failed") {
    if ((!fresh(now) && now - landing_start_ > 3) || now - landing_start_ > 90) {
      landing_state_ = "landing_failed";
      landing_detail_ = "landing feedback/operation timeout; pilot recovery required";
      return inhibit("EMERGENCY");
    }
  }
  if (enabled_ && tracking_seen_ && flag("navigation_fault"))
    return inhibit("HOLD");
  if (enabled_ && tracking_seen_ && now - direction_time_ > .3)
    return inhibit("HOLD");
  return {};
}
bool OperatorSession::navigation_permitted(double now) const {
  return enabled_ && !latched_ && fresh(now) && flag("ready") && flag("airborne") &&
         now - direction_time_ < .3;
}
Json OperatorSession::response(double now) const {
  bool safe = fresh(now) && flag("identity_verified") && flag("ground") &&
              feedback_.contains("armed") && feedback_["armed"] == false;
  return {{"status", fresh(now) ? feedback_
                                : Json{{"ready", false}, {"detail", "controller feedback stale"}}},
          {"enabled", enabled_},
          {"latched", latched_},
          {"confirmation", nullptr},
          {"safe_to_quit", safe},
          {"landing_state", landing_state_},
          {"landing_detail", landing_detail_}};
}
}  // namespace robotics::flight
