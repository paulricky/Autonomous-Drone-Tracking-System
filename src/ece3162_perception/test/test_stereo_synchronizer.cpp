#include <gtest/gtest.h>
#include "ece3162_perception/stereo_synchronizer.hpp"
using ece3162::perception::FrameStamp;
using ece3162::perception::StereoSynchronizer;
TEST(StereoSynchronizer, PairsWithinTolerance) {
  StereoSynchronizer sync(0.005, 4);
  EXPECT_TRUE(sync.push_left(FrameStamp{1, 1.000}));
  EXPECT_TRUE(sync.push_right(FrameStamp{8, 1.004}));
  const auto pair = sync.pop_pair(); ASSERT_TRUE(pair);
  EXPECT_EQ(pair->left.sequence, 1u); EXPECT_NEAR(pair->timestamp_delta_seconds, 0.004, 1e-12);
}
TEST(StereoSynchronizer, DropsOlderUnmatchedFrame) {
  StereoSynchronizer sync(0.005, 4);
  sync.push_left(FrameStamp{1, 1.0}); sync.push_right(FrameStamp{2, 1.1});
  EXPECT_FALSE(sync.pop_pair()); EXPECT_EQ(sync.stats().rejected, 1u);
}
TEST(StereoSynchronizer, RejectsNonMonotonicAndBoundsQueue) {
  StereoSynchronizer sync(0.005, 1);
  EXPECT_TRUE(sync.push_left(FrameStamp{1, 2.0}));
  EXPECT_FALSE(sync.push_left(FrameStamp{2, 1.0}));
  EXPECT_TRUE(sync.push_left(FrameStamp{3, 3.0}));
  EXPECT_EQ(sync.stats().non_monotonic, 1u); EXPECT_EQ(sync.stats().queue_overflow, 1u);
}
