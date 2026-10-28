#include <gtest/gtest.h>
#include <limits>

#include "robotics_core/command_gate.hpp"
using robotics::control::CommandGate;
using robotics::control::CommandSample;
using robotics::control::PhysicalHealth;
TEST(CommandGate, RejectsInvalidStaleReplayAndRequiresPhysicalHealth) {
  CommandGate gate;
  CommandSample c{10., 1, {0., 0., 1.}, false, "map"};
  EXPECT_TRUE(gate.check_common(c, 10.05).empty());
  auto bad = c;
  bad.acceleration[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gate.check_common(bad, 10.05).empty());
  EXPECT_FALSE(gate.check_common(c, 10.3).empty());
  gate.commit(c);
  EXPECT_EQ(gate.check_common(c, 10.05), "replayed command");
  c.stamp = 10.1;
  PhysicalHealth health;
  EXPECT_EQ(gate.check_physical(c, 10.12, health), "operator authorization absent");
  health.operator_authorized = true;
  health.controller_connected = true;
  EXPECT_EQ(gate.check_physical(c, 10.12, health),
            "physical protocol or configuration unverified");
  health.protocol_verified = health.configuration_verified = true;
  EXPECT_EQ(gate.check_physical(c, 10.12, health), "telemetry unavailable or stale");
  health.telemetry_valid = health.estimate_healthy = health.trajectory_valid = true;
  health.telemetry_stamp = health.estimate_stamp = health.trajectory_start = 10.1;
  health.trajectory_end = 11.0;
  health.trajectory_id = 1;
  health.operating_mode_known = true;
  EXPECT_TRUE(gate.check_physical(c, 10.12, health).empty());
  health.emergency_stop = true;
  EXPECT_EQ(gate.check_physical(c, 10.12, health), "emergency stop active");
}
