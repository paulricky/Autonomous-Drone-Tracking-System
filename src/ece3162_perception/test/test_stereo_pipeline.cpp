#include <gtest/gtest.h>

#include <cstdio>
#include <opencv2/imgproc.hpp>

#include "ece3162_perception/state_estimator.hpp"
#include "ece3162_perception/stereo_pipeline.hpp"
using namespace ece3162::perception;
TEST(VisualOdometry, RenderedSixDofMotion) {
  cv::setNumThreads(1);
  SyntheticStereo sensor;
  StereoConfig cfg;
  cfg.num_disparities = 64;
  StereoPipeline pipeline(sensor.calibration(), cfg);
  StereoVisualOdometry vo(240, 240, 159.5, 119.5);
  StateEstimator estimator;
  const std::vector<SceneBox> boxes = {{{2.5, -.4, 0}, {2.9, .4, 2}}};
  cv::Matx44d pose = camera_pose({0, 0, 1});
  for (int i = 0; i < 12; ++i) {
    pose = camera_pose({i * .02, i * .004, 1 + i * .002});
    auto frame = sensor.render_pose(1 + i * .1, pose, boxes);
    auto d = pipeline.process(frame.left, frame.right);
    auto result = vo.update(d.rectified_left, d.depth, frame.timestamp);
    auto state = estimator.update(result, frame.timestamp);
    if (i) {
      ASSERT_TRUE(result.valid) << result.failure;
      ASSERT_TRUE(state.usable(frame.timestamp));
      for (int j = 0; j < 3; ++j)
        EXPECT_NEAR(state.world_from_camera(j, 3), pose(j, 3), .04);
    }
  }
  auto blank = sensor.render_pose(2.3, pose, boxes, true);
  auto d = pipeline.process(blank.left, blank.right);
  auto result = vo.update(d.rectified_left, d.depth, 2.3);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(estimator.update(result, 2.3).usable(2.3));
}
TEST(StateEstimator, StaleJumpAndReset) {
  StateEstimator e;
  OdometryResult r;
  e.update(r, 1);
  r.valid = true;
  r.inliers = 100;
  r.previous_to_current(2, 3) = .02;
  auto s = e.update(r, 1.1);
  EXPECT_TRUE(s.usable(1.2));
  EXPECT_FALSE(s.usable(2));
  EXPECT_THROW(e.update(r, 1.1), std::invalid_argument);
  r.previous_to_current(2, 3) = 10;
  EXPECT_FALSE(e.update(r, 1.2).usable(1.2));
  e.reset(camera_pose({0, 0, 1}));
  EXPECT_EQ(e.state().quality, LocalizationQuality::initializing);
}
TEST(StereoCalibration, RejectsMalformedMatricesWithoutOpenCvAssertion) {
  auto c = SyntheticStereo().calibration();
  c.r_left.release();
  EXPECT_FALSE(c.valid());
  c = SyntheticStereo().calibration();
  c.k_left.convertTo(c.k_left, CV_32F);
  EXPECT_FALSE(c.valid());
}
TEST(VisualOdometry, RotationAndDepthValidity) {
  StereoVisualOdometry vo(240, 240);
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8U), depth(120, 160, CV_32F, 2.f);
  EXPECT_FALSE(vo.update(image, depth, 1).valid);
  EXPECT_FALSE(vo.update(image, depth, 1.1).valid);
  EXPECT_FALSE(vo.update(image, cv::Mat(3, 3, CV_32F), 1.2).valid);
}
TEST(StereoCalibration, RecoversSyntheticCheckerboardBaseline) {
  const cv::Size size(640, 480), pattern(7, 5);
  const double square = .04, baseline = .1;
  cv::Mat k = (cv::Mat_<double>(3, 3) << 500, 0, 319.5, 0, 500, 239.5, 0, 0, 1);
  cv::Mat board(300, 400, CV_8U);
  for (int y = 0; y < board.rows; ++y)
    for (int x = 0; x < board.cols; ++x)
      board.at<unsigned char>(y, x) = ((x / 50 + y / 50) % 2) ? 240 : 15;
  std::vector<cv::Point3f> corners{{0, 0, 0}, {.32f, 0, 0}, {.32f, .24f, 0}, {0, .24f, 0}};
  std::vector<cv::Point2f> source{{0, 0}, {399, 0}, {399, 299}, {0, 299}};
  std::vector<cv::Mat> left, right;
  for (int i = 0; i < 12; ++i) {
    cv::Vec3d rotation(.12 * (i % 3 - 1), .16 * (i % 4 - 1), .04 * (i % 5 - 2)),
        translation(-.16 + .025 * (i % 3 - 1), -.12 + .02 * (i % 4 - 1), .75 + .04 * (i % 4));
    for (int eye = 0; eye < 2; ++eye) {
      auto t = translation;
      t[0] -= eye * baseline;
      std::vector<cv::Point2f> pixels;
      cv::projectPoints(corners, rotation, t, k, cv::noArray(), pixels);
      cv::Mat image(size, CV_8U, cv::Scalar(180));
      cv::warpPerspective(board, image, cv::getPerspectiveTransform(source, pixels), size,
                          cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(180));
      (eye ? right : left).push_back(image);
    }
  }
  double usable = 0;
  auto c = calibrate_chessboard(left, right, pattern, square, &usable);
  ASSERT_TRUE(c.valid());
  EXPECT_GT(usable, .8);
  EXPECT_LT(c.rms_error, .6);
  EXPECT_NEAR(c.baseline(), baseline, .005);
  EXPECT_NEAR(c.k_left.at<double>(0, 0), 500, 30);
}
TEST(VisualOdometry, RecoversCameraRotation) {
  SyntheticStereo sensor;
  StereoConfig cfg;
  cfg.num_disparities = 64;
  StereoPipeline pipeline(sensor.calibration(), cfg);
  StereoVisualOdometry vo(240, 240, 159.5, 119.5);
  auto pose = camera_pose({0, 0, 1});
  auto first = sensor.render_pose(1, pose);
  auto d = pipeline.process(first.left, first.right);
  vo.update(d.rectified_left, d.depth, 1);
  cv::Matx33d rotation;
  cv::Rodrigues(cv::Vec3d(.005, .01, -.004), rotation);
  cv::Matx44d motion = cv::Matx44d::eye();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      motion(i, j) = rotation(i, j);
  auto second = sensor.render_pose(1.1, pose * motion);
  d = pipeline.process(second.left, second.right);
  auto r = vo.update(d.rectified_left, d.depth, 1.1);
  ASSERT_TRUE(r.valid) << r.failure;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NEAR(r.previous_to_current(i, j), motion(i, j), .005);
}
TEST(StereoCalibration, RoundTripAndRejectsInvalid) {
  SyntheticStereo synthetic;
  auto c = synthetic.calibration();
  ASSERT_TRUE(c.valid());
  const char* path = "/tmp/ece3162_synthetic_calibration.yaml";
  c.save(path);
  auto loaded = StereoCalibration::load(path);
  EXPECT_NEAR(loaded.baseline(), .1, 1e-12);
  EXPECT_EQ(loaded.image_size, cv::Size(320, 240));
  std::remove(path);
  c.translation = cv::Mat::zeros(3, 1, CV_64F);
  EXPECT_FALSE(c.valid());
}
TEST(StereoPipeline, SyntheticDepthAndCloud) {
  SyntheticStereo synthetic;
  auto frame = synthetic.render(1.0);
  StereoConfig cfg;
  cfg.num_disparities = 64;
  cfg.max_depth = 8;
  StereoPipeline pipeline(synthetic.calibration(), cfg);
  auto depth = pipeline.process(frame.left, frame.right);
  cv::Mat error;
  cv::absdiff(depth.depth, frame.ground_truth_depth, error);
  cv::Scalar mean = cv::mean(error, depth.valid_mask);
  const double valid = cv::countNonZero(depth.valid_mask) / double(depth.valid_mask.total());
  EXPECT_GT(valid, .45);
  EXPECT_LT(mean[0], .45);
  double cloud_ms{};
  auto cloud = pipeline.point_cloud(depth, 4, &cloud_ms);
  EXPECT_GT(cloud.size(), 1000u);
  EXPECT_GE(cloud_ms, 0);
}
TEST(StereoPipeline, RejectsDimensionMismatch) {
  SyntheticStereo synthetic;
  StereoPipeline pipeline(synthetic.calibration(), {});
  cv::Mat bad(10, 10, CV_8U);
  EXPECT_THROW(pipeline.process(bad, bad), std::invalid_argument);
}
TEST(VisualOdometry, ImageDerivedTranslationAndLoss) {
  const double fx = 240;
  StereoVisualOdometry vo(fx, fx);
  cv::Mat image(120, 160, CV_8U);
  cv::randu(image, 0, 255);
  cv::Mat depth(image.size(), CV_32F, 2.f);
  EXPECT_FALSE(vo.update(image, depth, 1).valid);
  cv::Mat shifted;
  cv::Mat transform = (cv::Mat_<double>(2, 3) << 1, 0, 4, 0, 1, 0);
  cv::warpAffine(image, shifted, transform, image.size());
  auto result = vo.update(shifted, depth, 1.1);
  ASSERT_TRUE(result.valid) << result.failure;
  EXPECT_NEAR(result.previous_to_current(0, 3), -4 * 2 / fx, .02);
  cv::Mat blank = cv::Mat::zeros(image.size(), CV_8U);
  auto lost = vo.update(blank, depth, 1.2);
  EXPECT_FALSE(lost.valid);
}
