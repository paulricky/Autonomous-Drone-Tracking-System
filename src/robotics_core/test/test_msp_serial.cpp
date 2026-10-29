#include <gtest/gtest.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include "robotics_core/msp_serial.hpp"

TEST(MspSerial, CorrelatesFragmentedReadOnlyReplyOnVirtualPort) {
  const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);
  ASSERT_EQ(::grantpt(master), 0);
  ASSERT_EQ(::unlockpt(master), 0);
  const char* path = ::ptsname(master);
  ASSERT_NE(path, nullptr);
  robotics::msp::SerialTelemetry transport(path, 115200);
  ASSERT_TRUE(transport.open_port());
  std::thread peer([master] {
    const auto send = [master](const std::uint8_t* bytes, std::size_t length) {
      std::size_t offset = 0;
      while (offset < length) {
        const ssize_t n = ::write(master, bytes + offset, length - offset);
        if (n <= 0) return false;
        offset += static_cast<std::size_t>(n);
      }
      return true;
    };
    std::uint8_t request[6];
    std::size_t received = 0;
    while (received < sizeof(request)) {
      const ssize_t n = ::read(master, request + received, sizeof(request) - received);
      if (n <= 0) return;
      received += static_cast<std::size_t>(n);
    }
    if (request[0] != '$' || request[4] != robotics::msp::kIdent) return;
    auto unrelated = robotics::msp::encode(robotics::msp::kStatus, {});
    unrelated[2] = '>';
    if (!send(unrelated.data(), unrelated.size())) return;
    auto bad = robotics::msp::encode(robotics::msp::kIdent, {1, 2, 3, 4, 5, 6, 7});
    bad[2] = '>';
    bad.back() ^= 1;
    if (!send(bad.data(), bad.size())) return;
    bad.back() ^= 1;
    for (auto byte : bad) {
      if (!send(&byte, 1)) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  const auto reply = transport.query(robotics::msp::kIdent, std::chrono::milliseconds(300));
  peer.join();
  ASSERT_TRUE(reply);
  EXPECT_EQ(reply->payload.size(), 7u);
  EXPECT_EQ(transport.invalid_packets(), 1u);
  EXPECT_THROW(transport.query(robotics::msp::kSetRawRc, std::chrono::milliseconds(100)),
               std::invalid_argument);
  transport.close_port();
  ::close(master);
}

TEST(MspSerial, DisconnectBetweenQueriesAllowsPersistentPathReconnect) {
  auto make_port = [] {
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0) return -1;
    if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
      ::close(master);
      return -1;
    }
    return master;
  };
  char path[] = "/tmp/msp-serial-reconnect-XXXXXX";
  const int temporary = ::mkstemp(path);
  ASSERT_GE(temporary, 0);
  ::close(temporary);
  ::unlink(path);
  const int first = make_port();
  ASSERT_GE(first, 0);
  ASSERT_EQ(::symlink(::ptsname(first), path), 0);
  robotics::msp::SerialTelemetry transport(path, 115200);
  ASSERT_TRUE(transport.open_port());
  ::close(first);
  EXPECT_FALSE(transport.query(robotics::msp::kStatus, std::chrono::milliseconds(20)));
  EXPECT_FALSE(transport.connected());
  // Remaining queries in this poll must report unavailable, without throwing.
  EXPECT_FALSE(transport.query(robotics::msp::kRawImu, std::chrono::milliseconds(20)));
  EXPECT_THROW(transport.query(robotics::msp::kSetRawRc, std::chrono::milliseconds(20)),
               std::invalid_argument);
  const int replacement = make_port();
  ASSERT_GE(replacement, 0);
  ::unlink(path);
  ASSERT_EQ(::symlink(::ptsname(replacement), path), 0);
  EXPECT_TRUE(transport.open_port());
  EXPECT_FALSE(transport.query(robotics::msp::kIdent, std::chrono::milliseconds(10)));
  EXPECT_TRUE(transport.connected());  // A bounded reply timeout permits another poll.
  transport.close_port();
  ::close(replacement);
  ::unlink(path);
}
