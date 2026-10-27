#include <gtest/gtest.h>

#include "robotics_core/msp.hpp"
TEST(Msp, EncodesKnownRawRcPacket) {
  const auto bytes = robotics::msp::encode_raw_rc({1500, 1500, 1500, 1000});
  ASSERT_EQ(bytes.size(), 14u);
  EXPECT_EQ(bytes[0], '$');
  EXPECT_EQ(bytes[3], 8);
  EXPECT_EQ(bytes[4], 200);
}
TEST(Msp, ParsesResponseAndRejectsBadChecksum) {
  robotics::msp::Parser parser;
  auto packet = robotics::msp::encode(108, {1, 2, 3});
  packet[2] = '>';
  std::optional<robotics::msp::Packet> result;
  for (auto byte : packet)
    if (auto item = parser.push(byte))
      result = item;
  ASSERT_TRUE(result);
  EXPECT_EQ(result->command, 108);
  EXPECT_EQ(result->payload, (std::vector<std::uint8_t>{1, 2, 3}));
  packet.back() ^= 1;
  for (auto byte : packet)
    parser.push(byte);
  EXPECT_EQ(parser.invalid_packets(), 1u);
}
TEST(Msp, RejectsUnsafeChannels) {
  EXPECT_THROW(robotics::msp::encode_raw_rc({1500, 1500, 1500, 2200}), std::invalid_argument);
}
