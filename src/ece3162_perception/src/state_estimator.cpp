#include "ece3162_perception/state_estimator.hpp"

#include <cmath>
namespace ece3162::perception {
bool EstimatedState::usable(double now, double age) const {
  return quality == LocalizationQuality::tracking && std::isfinite(now) && now >= timestamp &&
         now - timestamp <= age && cv::checkRange(cv::Mat(world_from_camera)) &&
         cv::checkRange(cv::Mat(velocity));
}
StateEstimator::StateEstimator(cv::Matx44d p) {
  reset(p);
}
void StateEstimator::reset(cv::Matx44d p) {
  if (!cv::checkRange(cv::Mat(p)))
    throw std::invalid_argument("non-finite initial pose");
  state_ = EstimatedState{};
  state_.world_from_camera = p;
  initialized_ = false;
  lost_ = false;
}
EstimatedState StateEstimator::update(const OdometryResult& o, double stamp) {
  if (!std::isfinite(stamp) || stamp <= 0 || (initialized_ && stamp <= state_.timestamp))
    throw std::invalid_argument("invalid estimator timestamp");
  if (!initialized_) {
    initialized_ = true;
    state_.timestamp = stamp;
    return state_;
  }
  const double dt = stamp - state_.timestamp;
  state_.timestamp = stamp;
  if (lost_ || !o.valid || dt > .5 || !cv::checkRange(cv::Mat(o.previous_to_current))) {
    lost_ = true;
    state_.quality = LocalizationQuality::lost;
    state_.velocity = {};
    return state_;
  }
  const cv::Matx44d pose = state_.world_from_camera * o.previous_to_current;
  cv::Vec3d v;
  for (int i = 0; i < 3; ++i)
    v[i] = (pose(i, 3) - state_.world_from_camera(i, 3)) / dt;
  cv::Matx33d rotation;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      rotation(i, j) = o.previous_to_current(i, j);
  cv::Vec3d r;
  cv::Rodrigues(rotation, r);
  if (cv::norm(v) > 3.0 || cv::norm(r) / dt > 3.0) {
    lost_ = true;
    state_.quality = LocalizationQuality::lost;
    return state_;
  }
  const double alpha = dt / (.15 + dt);
  state_.velocity = (1 - alpha) * state_.velocity + alpha * v;
  state_.world_from_camera = pose;
  state_.quality = LocalizationQuality::tracking;
  // Heuristic accumulated uncertainty, not a calibrated probabilistic covariance.
  for (int i = 0; i < 6; ++i)
    state_.covariance(i, i) += std::max(1e-7, o.reprojection_error * o.reprojection_error * 1e-6);
  return state_;
}
}  // namespace ece3162::perception
