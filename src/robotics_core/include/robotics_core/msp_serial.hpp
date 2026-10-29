#pragma once
#include <chrono>
#include <optional>
#include <string>

#include "robotics_core/msp.hpp"

namespace robotics::msp {
// Read-only MSP v1 transport. Only legacy archive query IDs are accepted.
class SerialTelemetry {
 public:
  SerialTelemetry(std::string device, int baud);
  ~SerialTelemetry();
  SerialTelemetry(const SerialTelemetry&) = delete;
  SerialTelemetry& operator=(const SerialTelemetry&) = delete;
  bool open_port();
  void close_port() noexcept;
  bool connected() const noexcept { return fd_ >= 0; }
  // Disconnection/timeouts return nullopt; unsupported commands or invalid timeouts throw.
  std::optional<Packet> query(std::uint8_t command, std::chrono::milliseconds timeout);
  std::size_t invalid_packets() const noexcept { return parser_.invalid_packets(); }
 private:
  std::string device_;
  int baud_;
  int fd_{-1};
  Parser parser_;
};
}  // namespace robotics::msp
