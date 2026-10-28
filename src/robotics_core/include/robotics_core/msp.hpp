#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace robotics::msp {
constexpr std::uint8_t kSetRawRc = 200;
struct Packet {
  std::uint8_t command{};
  std::vector<std::uint8_t> payload;
  bool error{};
};
constexpr std::uint8_t kIdent = 100;
constexpr std::uint8_t kStatus = 101;
constexpr std::uint8_t kRawImu = 102;
constexpr std::uint8_t kRc = 105;
constexpr std::uint8_t kAttitude = 108;
constexpr std::uint8_t kAnalog = 110;
struct Ident {
  std::uint8_t firmware_version{}, vehicle_type{}, msp_version{};
  std::uint32_t capability{};
};
struct Status {
  std::uint16_t cycle_time_us{}, i2c_errors{}, sensor_flags{};
  std::uint32_t mode_flags{};
  std::uint8_t profile{};
};
struct Analog {
  std::uint8_t battery_raw{};
  std::uint16_t power_meter{}, rssi{}, current_raw{};
};
std::optional<Ident> decode_ident(const Packet&);
std::optional<Status> decode_status(const Packet&);
std::optional<Analog> decode_analog(const Packet&);
std::vector<std::uint8_t> encode(std::uint8_t command,
                                 const std::vector<std::uint8_t>& payload = {});
std::vector<std::uint8_t> encode_raw_rc(
    const std::array<std::uint16_t, 4>& roll_pitch_yaw_throttle);
class Parser {
 public:
  std::optional<Packet> push(std::uint8_t byte);
  void discard_partial() noexcept { reset(); }
  std::size_t invalid_packets() const noexcept {
    return invalid_packets_;
  }

 private:
  enum class State { kDollar, kM, kDirection, kLength, kCommand, kPayload, kChecksum };
  void reset() noexcept;
  State state_{State::kDollar};
  std::uint8_t length_{};
  std::uint8_t checksum_{};
  bool error_{};
  Packet packet_{};
  std::size_t invalid_packets_{};
};
}  // namespace robotics::msp
