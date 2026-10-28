#include "robotics_core/msp.hpp"

#include <stdexcept>

namespace robotics::msp {
namespace {
std::uint16_t u16(const std::vector<std::uint8_t>& p, std::size_t at) {
  return std::uint16_t(p[at]) | (std::uint16_t(p[at + 1]) << 8);
}
std::uint32_t u32(const std::vector<std::uint8_t>& p, std::size_t at) {
  return std::uint32_t(u16(p, at)) | (std::uint32_t(u16(p, at + 2)) << 16);
}
}
std::optional<Ident> decode_ident(const Packet& p) {
  if (p.error || p.command != kIdent || p.payload.size() != 7) return std::nullopt;
  return Ident{p.payload[0], p.payload[1], p.payload[2], u32(p.payload, 3)};
}
std::optional<Status> decode_status(const Packet& p) {
  if (p.error || p.command != kStatus || p.payload.size() != 11) return std::nullopt;
  return Status{u16(p.payload, 0), u16(p.payload, 2), u16(p.payload, 4),
                u32(p.payload, 6), p.payload[10]};
}
std::optional<Analog> decode_analog(const Packet& p) {
  if (p.error || p.command != kAnalog || p.payload.size() != 7) return std::nullopt;
  return Analog{p.payload[0], u16(p.payload, 1), u16(p.payload, 3), u16(p.payload, 5)};
}
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
  error_ = false;
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
      error_ = byte == '!';
      state_ = (byte == '>' || error_) ? State::kLength : State::kDollar;
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
        complete.error = error_;
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
