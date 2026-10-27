#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace ece3162::msp {
constexpr std::uint8_t kSetRawRc = 200;
struct Packet {
  std::uint8_t command{};
  std::vector<std::uint8_t> payload;
};
std::vector<std::uint8_t> encode(std::uint8_t command,
                                 const std::vector<std::uint8_t>& payload = {});
std::vector<std::uint8_t> encode_raw_rc(
    const std::array<std::uint16_t, 4>& roll_pitch_yaw_throttle);
class Parser {
 public:
  std::optional<Packet> push(std::uint8_t byte);
  std::size_t invalid_packets() const noexcept {
    return invalid_packets_;
  }

 private:
  enum class State { kDollar, kM, kDirection, kLength, kCommand, kPayload, kChecksum };
  void reset() noexcept;
  State state_{State::kDollar};
  std::uint8_t length_{};
  std::uint8_t checksum_{};
  Packet packet_{};
  std::size_t invalid_packets_{};
};
}  // namespace ece3162::msp
