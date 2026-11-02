#include <gtest/gtest.h>

#include "robotics_core/controller_interface.hpp"
#include "robotics_core/operator_session.hpp"
using namespace robotics::flight;
static Json healthy(bool armed = false) {
  return {{"connected", true}, {"ready", true},          {"identity_verified", true},
          {"ground", true},    {"armed", armed},         {"airborne", false},
          {"mode", "POSCTL"},  {"takeoff_allowed", true}};
}
TEST(OperatorProtocol, AuthenticationExpirationAndRawPythonEncoding) {
  std::string key = "0123456789abcdef0123456789abcdef";
  std::string payload = R"({"command":"EMERGENCY","seq":1,"time":1000.0})";
  auto packet = authenticate(key, payload) + "\n" + payload;
  EXPECT_EQ(decode_operator_packet(key, packet, 1000)["command"], "EMERGENCY");
  EXPECT_THROW(decode_operator_packet(key, packet, 1001), std::invalid_argument);
  packet.back() = 'x';
  EXPECT_THROW(decode_operator_packet(key, packet, 1000), std::invalid_argument);
}
TEST(OperatorAuthorization, SinglePressTakeoffRequiresSeparateArmingAndCannotRepeat) {
  OperatorSession mission("measured");
  mission.connected(1);
  mission.observe(healthy(), 1);
  EXPECT_TRUE(mission.handle({{"command", "TAKEOFF"}}, 1).operation.empty());
  mission.handle({{"command", "ENABLE"}}, 1);
  EXPECT_TRUE(mission.handle({{"command", "TAKEOFF"}}, 1).operation.empty());
  EXPECT_EQ(mission.handle({{"command", "ARM"}}, 1).operation, "ARM");
  EXPECT_TRUE(mission.handle({{"command", "TAKEOFF"}}, 1).operation.empty());
  mission.observe(healthy(true), 1.1);
  EXPECT_EQ(mission.handle({{"command", "TAKEOFF"}}, 1.1).operation, "TAKEOFF");
  EXPECT_TRUE(mission.handle({{"command", "TAKEOFF"}}, 1.1).operation.empty());
  mission.connected(1.2);
  mission.handle({{"command", "ENABLE"}}, 1.2);
  EXPECT_TRUE(mission.handle({{"command", "TAKEOFF"}}, 1.2).operation.empty());
}
TEST(OperatorAuthorization, StaleReadinessOrEmergencyBlocksTakeoff) {
  OperatorSession mission("measured");
  mission.connected(1);
  mission.observe(healthy(), 1);
  mission.handle({{"command", "ENABLE"}}, 1);
  mission.handle({{"command", "ARM"}}, 1);
  mission.observe(healthy(true), 1.1);
  EXPECT_TRUE(mission.handle({{"command", "TAKEOFF"}}, 2).operation.empty());
  mission.observe(healthy(true), 2);
  EXPECT_EQ(mission.handle({{"command", "EMERGENCY"}}, 2).operation, "EMERGENCY");
  EXPECT_TRUE(mission.handle({{"command", "ENABLE"}}, 2).operation.empty());
  EXPECT_TRUE(mission.handle({{"command", "TAKEOFF"}}, 2).operation.empty());
}
TEST(OperatorWatchdog, LostIntentHoldsAndLostConnectionLatches) {
  OperatorSession mission("measured");
  mission.connected(1);
  auto state = healthy(true);
  state["ground"] = false;
  state["airborne"] = true;
  state["mode"] = "OFFBOARD";
  mission.observe(state, 1);
  mission.handle({{"command", "ENABLE"}}, 1);
  Json direction = {{"command", "DIRECTION"},
                    {"direction", {1., 0., 0.}},
                    {"frame", "wrong"},
                    {"operator_enabled", true}};
  EXPECT_FALSE(mission.handle(direction, 1).direction);
  direction["frame"] = "measured";
  EXPECT_TRUE(mission.handle(direction, 1).direction);
  EXPECT_TRUE(mission.navigation_permitted(1.1));
  EXPECT_EQ(mission.tick(1.31).operation, "HOLD");
  EXPECT_FALSE(mission.navigation_permitted(1.31));
  EXPECT_EQ(mission.tick(1.7).operation, "EMERGENCY");
  EXPECT_TRUE(mission.latched());
}
TEST(OperatorLanding, RequiresAuthoritativeGroundAndDisarming) {
  OperatorSession mission("x");
  mission.connected(1);
  auto state = healthy(true);
  state["ground"] = false;
  state["airborne"] = true;
  mission.observe(state, 1);
  mission.handle({{"command", "LAND"}}, 1);
  EXPECT_FALSE(mission.response(1)["safe_to_quit"].get<bool>());
  state["ground"] = true;
  mission.observe(state, 1.1);
  EXPECT_EQ(mission.response(1.1)["landing_state"], "landed");
  EXPECT_EQ(mission.handle({{"command", "DISARM"}}, 1.1).operation, "DISARM");
  state["armed"] = false;
  mission.observe(state, 1.2);
  EXPECT_TRUE(mission.response(1.2)["safe_to_quit"].get<bool>());
}
TEST(ControllerBoundary, UnverifiedMspCannotExecutePhysicalCommands) {
  UnverifiedMspBackend backend;
  EXPECT_FALSE(backend.capabilities().takeoff);
  EXPECT_FALSE(backend.capabilities().land);
  EXPECT_FALSE(backend.submit_motion({1, {0, 0, 0}, false, "map"}));
}
