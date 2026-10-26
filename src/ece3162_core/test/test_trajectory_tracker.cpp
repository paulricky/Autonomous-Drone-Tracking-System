#include <gtest/gtest.h>
#include <cmath>
#include "ece3162_core/trajectory_tracker.hpp"
TEST(TrajectoryTracker, SaturatesVectorNorm) {
  ece3162::control::TrajectoryTracker tracker;
  ece3162::control::Reference ref; ref.position = {1,1,1};
  ece3162::control::State state; state.valid = true; state.stamp = 1.0;
  const auto cmd = tracker.update(1.1, ref, state); ASSERT_TRUE(cmd); EXPECT_TRUE(cmd->saturated);
  double n2 = 0; for (auto x : cmd->acceleration) n2 += x*x; EXPECT_NEAR(std::sqrt(n2), 2.0, 1e-12);
}
TEST(TrajectoryTracker, RejectsStaleState) {
  ece3162::control::TrajectoryTracker tracker; ece3162::control::Reference ref;
  ece3162::control::State state; state.valid = true; state.stamp = 0.0;
  EXPECT_FALSE(tracker.update(1.0, ref, state)); EXPECT_EQ(tracker.failure_reason(), "invalid or stale localization");
}
