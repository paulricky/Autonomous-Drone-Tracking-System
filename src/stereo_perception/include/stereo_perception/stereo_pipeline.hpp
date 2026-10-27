#pragma once
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace robotics::perception {
enum class TimestampSource { kCamera, kDriverMonotonic, kHostMonotonic, kRecorded };
struct ImageFrame {
  cv::Mat image;
  double capture_time{}, receive_time{};
  std::uint64_t sequence{};
  std::string source_id;
  TimestampSource timestamp_source{TimestampSource::kHostMonotonic};
};
struct CameraConfig {
  std::string device, source_id;
  int width{640}, height{480};
  double fps{20}, exposure{-1}, gain{-1};
  int fourcc{};
};
struct AcquisitionStats {
  std::size_t frames{}, failures{}, dropped{};
};
class UvcCamera {
 public:
  explicit UvcCamera(CameraConfig);
  ~UvcCamera();
  UvcCamera(const UvcCamera&) = delete;
  UvcCamera& operator=(const UvcCamera&) = delete;
  std::optional<ImageFrame> read();
  bool opened() const noexcept {
    return capture_.isOpened();
  }
  const AcquisitionStats& stats() const noexcept {
    return stats_;
  }
  const std::vector<std::string>& unsupported_controls() const noexcept {
    return unsupported_;
  }

 private:
  CameraConfig config_;
  cv::VideoCapture capture_;
  AcquisitionStats stats_;
  std::vector<std::string> unsupported_;
  std::uint64_t sequence_{};
  double last_receive_{};
};
struct StereoCalibration {
  cv::Mat k_left, d_left, k_right, d_right, rotation, translation, r_left, r_right, p_left, p_right,
      q;
  cv::Size image_size;
  double rms_error{};
  std::string provenance;
  bool valid(std::string* reason = nullptr) const;
  double baseline() const;
  static StereoCalibration load(const std::string&);
  void save(const std::string&) const;
};
StereoCalibration calibrate_chessboard(const std::vector<cv::Mat>& left,
                                       const std::vector<cv::Mat>& right, cv::Size pattern,
                                       double square_size_m, double* usable_fraction = nullptr);
struct StereoConfig {
  int min_disparity{}, num_disparities{128}, block_size{5}, disp12_max_diff{1}, uniqueness_ratio{8},
      speckle_window{50}, speckle_range{2};
  double min_depth{.2}, max_depth{10}, disparity_sigma{.5};
};
struct DepthFrame {
  cv::Mat rectified_left, rectified_right, disparity, depth, valid_mask, confidence;
  double rectify_ms{}, disparity_ms{}, depth_ms{};
};
class StereoPipeline {
 public:
  StereoPipeline(StereoCalibration, StereoConfig = {});
  DepthFrame process(const cv::Mat&, const cv::Mat&);
  std::vector<cv::Point3f> point_cloud(const DepthFrame&, int stride = 1,
                                       double* milliseconds = nullptr) const;
  const StereoCalibration& calibration() const noexcept {
    return calibration_;
  }

 private:
  StereoCalibration calibration_;
  StereoConfig config_;
  cv::Mat lmx_, lmy_, rmx_, rmy_;
  cv::Ptr<cv::StereoSGBM> matcher_;
};
struct SyntheticConfig {
  cv::Size size{320, 240};
  double fx{240}, fy{240}, cx{159.5}, cy{119.5}, baseline{.1};
  unsigned seed{7};
  double noise_std{};
};
struct SyntheticFrame {
  cv::Mat left, right, ground_truth_depth;
  double timestamp{};
};
struct SceneBox {
  cv::Vec3d minimum, maximum;
};
// map: x forward, y left, z up; camera optical: x right, y down, z forward.
cv::Matx44d camera_pose(const cv::Vec3d& position);
class SyntheticStereo {
 public:
  explicit SyntheticStereo(SyntheticConfig = {});
  SyntheticFrame render(double timestamp, double foreground_depth = 2.,
                        double background_depth = 5.) const;
  SyntheticFrame render_pose(double timestamp, const cv::Matx44d& world_from_camera,
                             const std::vector<SceneBox>& obstacles = {},
                             bool low_texture = false) const;
  StereoCalibration calibration() const;

 private:
  SyntheticConfig config_;
  cv::Mat texture_;
};
struct OdometryResult {
  bool valid{};
  cv::Matx44d previous_to_current{cv::Matx44d::eye()};
  int features{}, inliers{};
  double reprojection_error{}, processing_ms{};
  std::string failure;
};
// previous_to_current stores previous-camera-from-current-camera (camera displacement).
class StereoVisualOdometry {
 public:
  StereoVisualOdometry(double fx, double fy, double cx = -1, double cy = -1)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {
    if (fx <= 0 || fy <= 0)
      throw std::invalid_argument("invalid odometry intrinsics");
  }
  OdometryResult update(const cv::Mat& rectified_left, const cv::Mat& depth, double timestamp);
  void reset();

 private:
  cv::Mat previous_image_, previous_depth_;
  double previous_stamp_{}, fx_, fy_, cx_, cy_;
};
}  // namespace robotics::perception
