#pragma once
#include "ece3162_perception/stereo_pipeline.hpp"

namespace ece3162::perception {
enum class LocalizationQuality { initializing, tracking, lost };
struct EstimatedState {
  cv::Matx44d world_from_camera{cv::Matx44d::eye()};
  cv::Vec3d velocity{};
  cv::Matx<double, 6, 6> covariance{cv::Matx<double, 6, 6>::eye() * .001};
  double timestamp{};
  LocalizationQuality quality{LocalizationQuality::initializing};
  bool usable(double now, double max_age = .5) const;
};
// Stereo-only relative odometry, initialized from configured camera extrinsics.
// No simulator state, IMU, global correction, or loop closure enters this class.
class StateEstimator {
 public:
  explicit StateEstimator(cv::Matx44d initial_pose = camera_pose({0, 0, 1}));
  EstimatedState update(const OdometryResult&, double timestamp);
  void reset(cv::Matx44d initial_pose);
  const EstimatedState& state() const {
    return state_;
  }

 private:
  EstimatedState state_;
  bool initialized_{};
  bool lost_{};
};
}  // namespace ece3162::perception
