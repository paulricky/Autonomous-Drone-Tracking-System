#include <gtest/gtest.h>

#include "robotics_core/msp.hpp"

TEST(Msp, ReadOnlyApiIdentityLayoutsRejectMalformedReplies) {
  using namespace robotics::msp;
  const auto api = decode_api_version({kApiVersion, {0, 1, 46}, false});
  ASSERT_TRUE(api);
  EXPECT_EQ(api->major, 1);
  EXPECT_EQ(api->minor, 46);
  EXPECT_FALSE(decode_api_version({kApiVersion, {0, 1}, false}));
  EXPECT_FALSE(decode_api_version({kApiVersion, {0, 1, 46}, true}));
  EXPECT_FALSE(decode_api_version({kIdent, {0, 1, 46}, false}));
  EXPECT_EQ(decode_ascii_prefix({kFcVariant, {'B', 'T', 'F', 'L'}, false}, kFcVariant, 4, true),
            "BTFL");
  EXPECT_FALSE(decode_ascii_prefix({kFcVariant, {'B', 0, 'F', 'L'}, false}, kFcVariant, 4, true));
  EXPECT_FALSE(
      decode_ascii_prefix({kFcVariant, {'B', 'T', 'F', 'L', 0}, false}, kFcVariant, 4, true));
  EXPECT_EQ(
      decode_ascii_prefix({kBoardInfo, {'S', '4', '1', '1', 0, 0}, false}, kBoardInfo, 4, false),
      "S411");
}
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
TEST(Msp, DecodesReadOnlyLegacyFieldsAndRejectsErrors) {
  robotics::msp::Parser parser;
  auto bytes = robotics::msp::encode(robotics::msp::kIdent, {1, 2, 3, 4, 5, 6, 7});
  bytes[2] = '>';
  std::optional<robotics::msp::Packet> packet;
  for (auto byte : bytes)
    if (auto item = parser.push(byte))
      packet = item;
  ASSERT_TRUE(packet);
  auto ident = robotics::msp::decode_ident(*packet);
  ASSERT_TRUE(ident);
  EXPECT_EQ(ident->capability, 0x07060504u);
  bytes[2] = '!';
  for (auto byte : bytes)
    if (auto item = parser.push(byte))
      packet = item;
  ASSERT_TRUE(packet->error);
  EXPECT_FALSE(robotics::msp::decode_ident(*packet));
  bytes[2] = '<';
  packet.reset();
  for (auto byte : bytes)
    if (auto item = parser.push(byte))
      packet = item;
  EXPECT_FALSE(packet);
}
