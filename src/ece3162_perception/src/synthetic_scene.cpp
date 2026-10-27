#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

#include "ece3162_perception/stereo_pipeline.hpp"

namespace ece3162::perception {
cv::Matx44d camera_pose(const cv::Vec3d& p) {
  return {0, 0, 1, p[0], -1, 0, 0, p[1], 0, -1, 0, p[2], 0, 0, 0, 1};
}
SyntheticFrame SyntheticStereo::render_pose(double timestamp, const cv::Matx44d& pose,
                                            const std::vector<SceneBox>& boxes,
                                            bool low_texture) const {
  if (!std::isfinite(timestamp) || !cv::checkRange(cv::Mat(pose)))
    throw std::invalid_argument("invalid render pose/time");
  SyntheticFrame out;
  out.timestamp = timestamp;
  out.left = cv::Mat(config_.size, CV_8U);
  out.right = cv::Mat(config_.size, CV_8U);
  out.ground_truth_depth = cv::Mat(config_.size, CV_32F);
  cv::Matx33d rotation;
  cv::Vec3d origin;
  for (int i = 0; i < 3; ++i) {
    origin[i] = pose(i, 3);
    for (int j = 0; j < 3; ++j)
      rotation(i, j) = pose(i, j);
  }
  // Fixed world textures: each eye and each time samples the same metric surface.
  auto texture = [&](const cv::Vec3d& p, int normal) {
    if (low_texture)
      return 127.;
    int a = (normal + 1) % 3, b = (normal + 2) % 3;
    double u = p[a] * 70 + 10000, v = p[b] * 70 + 10000;
    int iu = int(std::floor(u)), iv = int(std::floor(v));
    double du = u - iu, dv = v - iv;
    auto sample = [&](int x, int y) {
      return double(
          texture_.at<unsigned char>((y % texture_.rows + texture_.rows) % texture_.rows,
                                     (x % texture_.cols + texture_.cols) % texture_.cols));
    };
    return (1 - dv) * ((1 - du) * sample(iu, iv) + du * sample(iu + 1, iv)) +
           dv * ((1 - du) * sample(iu, iv + 1) + du * sample(iu + 1, iv + 1));
  };
  for (int eye = 0; eye < 2; ++eye) {
    cv::Vec3d sensor = origin + rotation * cv::Vec3d(eye * config_.baseline, 0, 0);
    cv::Mat& image = eye ? out.right : out.left;
    for (int y = 0; y < image.rows; ++y)
      for (int x = 0; x < image.cols; ++x) {
        cv::Vec3d ray =
            rotation * cv::Vec3d((x - config_.cx) / config_.fx, (y - config_.cy) / config_.fy, 1);
        double distance = 100;
        int normal = 0;
        // Room surfaces: background, side walls, floor and ceiling.
        const double lower[3] = {-2, -3, 0}, upper[3] = {7, 3, 3};
        for (int axis = 0; axis < 3; ++axis)
          for (double plane : {lower[axis], upper[axis]}) {
            if (std::abs(ray[axis]) < 1e-9)
              continue;
            double t = (plane - sensor[axis]) / ray[axis];
            if (t > .05 && t < distance) {
              distance = t;
              normal = axis;
            }
          }
        for (const auto& box : boxes) {
          double near = -1e9, far = 1e9;
          int face = 0;
          for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(ray[axis]) < 1e-9) {
              if (sensor[axis] < box.minimum[axis] || sensor[axis] > box.maximum[axis]) {
                far = -1;
                break;
              }
              continue;
            }
            double a = (box.minimum[axis] - sensor[axis]) / ray[axis],
                   b = (box.maximum[axis] - sensor[axis]) / ray[axis];
            if (a > b)
              std::swap(a, b);
            if (a > near) {
              near = a;
              face = axis;
            }
            far = std::min(far, b);
          }
          if (near > .05 && near <= far && near < distance) {
            distance = near;
            normal = face;
          }
        }
        image.at<unsigned char>(y, x) =
            cv::saturate_cast<unsigned char>(texture(sensor + distance * ray, normal));
        if (!eye)
          out.ground_truth_depth.at<float>(y, x) = float(distance);
      }
    if (config_.noise_std > 0) {
      cv::Mat noise(image.size(), CV_16S), temp;
      cv::RNG rng(config_.seed + unsigned(timestamp * 1000) + eye * 1237);
      rng.fill(noise, cv::RNG::NORMAL, 0, config_.noise_std);
      image.convertTo(temp, CV_16S);
      temp += noise;
      temp.convertTo(image, CV_8U);
    }
  }
  return out;
}
}  // namespace ece3162::perception
