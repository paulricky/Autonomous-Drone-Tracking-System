#include "robotics_core/msp_serial.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>

namespace robotics::msp {
SerialTelemetry::SerialTelemetry(std::string device, int baud)
    : device_(std::move(device)), baud_(baud) {
  if (device_.empty() || (baud_ != 115200 && baud_ != 57600))
    throw std::invalid_argument("serial path and supported explicit baud required");
}
SerialTelemetry::~SerialTelemetry() {
  close_port();
}
bool SerialTelemetry::open_port() {
  close_port();
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0)
    return false;
  termios settings{};
  if (::tcgetattr(fd_, &settings) != 0) {
    close_port();
    return false;
  }
  ::cfmakeraw(&settings);
  settings.c_iflag &= ~(IXON | IXOFF | IXANY);
  const speed_t speed = baud_ == 115200 ? B115200 : B57600;
  ::cfsetispeed(&settings, speed);
  ::cfsetospeed(&settings, speed);
  settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  settings.c_cflag |= CS8 | CLOCAL | CREAD;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (::tcsetattr(fd_, TCSANOW, &settings) != 0) {
    close_port();
    return false;
  }
  parser_ = Parser{};
  return true;
}
void SerialTelemetry::close_port() noexcept {
  if (fd_ >= 0)
    ::close(fd_);
  fd_ = -1;
}
std::optional<Packet> SerialTelemetry::query(std::uint8_t command,
                                             std::chrono::milliseconds timeout) {
  if (command != kApiVersion && command != kFcVariant && command != kFcVersion &&
      command != kBoardInfo && command != kIdent && command != kStatus && command != kRawImu &&
      command != kRc && command != kAttitude && command != kAnalog)
    throw std::invalid_argument("MSP query is not on the read-only allowlist");
  if (timeout.count() <= 0 || timeout.count() > 1000)
    throw std::invalid_argument("serial timeout invalid");
  // Disconnection is an expected transport event, including between queries
  // in one telemetry poll. Let the owner publish unavailable data and reconnect.
  if (!connected())
    return std::nullopt;
  // MSP v1 has no transaction ID. Drop a timed-out partial reply and any
  // already queued bytes before issuing the next single outstanding query.
  parser_.discard_partial();
  if (::tcflush(fd_, TCIFLUSH) != 0) {
    close_port();
    return std::nullopt;
  }
  const auto request = encode(command);
  auto deadline = std::chrono::steady_clock::now() + timeout;
  std::size_t written = 0;
  while (written < request.size()) {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                         deadline - std::chrono::steady_clock::now())
                         .count();
    if (remaining <= 0)
      return std::nullopt;
    pollfd event{fd_, POLLOUT, 0};
    const int polled = ::poll(&event, 1, static_cast<int>(remaining));
    if (polled < 0 && errno == EINTR)
      continue;
    if (polled <= 0 || (event.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      if (polled > 0)
        close_port();
      return std::nullopt;
    }
    const ssize_t n = ::write(fd_, request.data() + written, request.size() - written);
    if (n > 0)
      written += static_cast<std::size_t>(n);
    else if (n < 0 && errno != EAGAIN && errno != EINTR) {
      close_port();
      return std::nullopt;
    }
  }
  while (std::chrono::steady_clock::now() < deadline) {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                         deadline - std::chrono::steady_clock::now())
                         .count();
    pollfd event{fd_, POLLIN, 0};
    const int polled = ::poll(&event, 1, static_cast<int>(remaining));
    if (polled < 0 && errno == EINTR)
      continue;
    if (polled <= 0)
      return std::nullopt;
    if (event.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      close_port();
      return std::nullopt;
    }
    std::uint8_t bytes[256];
    const ssize_t n = ::read(fd_, bytes, sizeof(bytes));
    if (n == 0) {
      close_port();
      return std::nullopt;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      close_port();
      return std::nullopt;
    }
    for (ssize_t i = 0; i < n; ++i)
      if (auto packet = parser_.push(bytes[i]); packet && packet->command == command)
        return packet;
  }
  return std::nullopt;
}
}  // namespace robotics::msp
