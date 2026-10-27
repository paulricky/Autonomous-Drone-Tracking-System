#include "robotics_core/msp.hpp"

#include <stdexcept>

namespace robotics::msp {
std::vector<std::uint8_t> encode(const std::uint8_t command,
                                 const std::vector<std::uint8_t>& payload) {
  if (payload.size() > 255)
    throw std::invalid_argument("MSP v1 payload exceeds 255 bytes");
  const auto length = static_cast<std::uint8_t>(payload.size());
  std::uint8_t checksum = length ^ command;
  std::vector<std::uint8_t> result{'$', 'M', '<', length, command};
  result.reserve(payload.size() + 6);
  for (const auto byte : payload) {
    result.push_back(byte);
    checksum ^= byte;
  }
  result.push_back(checksum);
  return result;
}
std::vector<std::uint8_t> encode_raw_rc(const std::array<std::uint16_t, 4>& channels) {
  std::vector<std::uint8_t> payload;
  payload.reserve(8);
  for (const auto value : channels) {
    if (value < 900 || value > 2100)
      throw std::invalid_argument("RC channel outside verified bound");
    payload.push_back(static_cast<std::uint8_t>(value & 0xff));
    payload.push_back(static_cast<std::uint8_t>(value >> 8));
  }
  return encode(kSetRawRc, payload);
}
void Parser::reset() noexcept {
  state_ = State::kDollar;
  length_ = checksum_ = 0;
  packet_ = {};
}
std::optional<Packet> Parser::push(const std::uint8_t byte) {
  switch (state_) {
    case State::kDollar:
      if (byte == '$')
        state_ = State::kM;
      break;
    case State::kM:
      state_ = byte == 'M' ? State::kDirection : State::kDollar;
      break;
    case State::kDirection:
      state_ = (byte == '>' || byte == '!') ? State::kLength : State::kDollar;
      break;
    case State::kLength:
      length_ = byte;
      checksum_ = byte;
      packet_.payload.clear();
      state_ = State::kCommand;
      break;
    case State::kCommand:
      packet_.command = byte;
      checksum_ ^= byte;
      state_ = length_ ? State::kPayload : State::kChecksum;
      break;
    case State::kPayload:
      packet_.payload.push_back(byte);
      checksum_ ^= byte;
      if (packet_.payload.size() == length_)
        state_ = State::kChecksum;
      break;
    case State::kChecksum:
      if (byte == checksum_) {
        auto complete = packet_;
        reset();
        return complete;
      }
      ++invalid_packets_;
      reset();
      break;
  }
  return std::nullopt;
}
}  // namespace robotics::msp
