#include "ece3162_perception/stereo_pipeline.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdexcept>

namespace ece3162::perception {
namespace {
double monotonic() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double ms(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}
}  // namespace
UvcCamera::UvcCamera(CameraConfig c) : config_(std::move(c)) {
  if (config_.device.empty() || config_.source_id.empty() || config_.width <= 0 ||
      config_.height <= 0 || config_.fps <= 0)
    throw std::invalid_argument("invalid camera configuration");
  capture_.open(config_.device, cv::CAP_V4L2);
  if (!capture_.isOpened())
    throw std::runtime_error("cannot open UVC camera " + config_.device);
  if (!capture_.set(cv::CAP_PROP_BUFFERSIZE, 1))
    unsupported_.push_back("buffer_size");
  if (!capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width))
    unsupported_.push_back("width");
  if (!capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height))
    unsupported_.push_back("height");
  if (!capture_.set(cv::CAP_PROP_FPS, config_.fps))
    unsupported_.push_back("fps");
  if (config_.fourcc && !capture_.set(cv::CAP_PROP_FOURCC, config_.fourcc))
    unsupported_.push_back("pixel_format");
  if (config_.exposure >= 0 && !capture_.set(cv::CAP_PROP_EXPOSURE, config_.exposure))
    unsupported_.push_back("exposure");
  if (config_.gain >= 0 && !capture_.set(cv::CAP_PROP_GAIN, config_.gain))
    unsupported_.push_back("gain");
}
UvcCamera::~UvcCamera() {
  capture_.release();
}
std::optional<ImageFrame> UvcCamera::read() {
  cv::Mat image;
  if (!capture_.read(image) || image.empty()) {
    ++stats_.failures;
    return std::nullopt;
  }
  const double time = monotonic();
  if (last_receive_ > 0 && time - last_receive_ > 1.5 / config_.fps)
    stats_.dropped += static_cast<std::size_t>((time - last_receive_) * config_.fps) - 1;
  last_receive_ = time;
  if (image.size() != cv::Size(config_.width, config_.height)) {
    ++stats_.failures;
    return std::nullopt;
  }
  if (image.channels() == 3)
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  else if (image.channels() == 4)
    cv::cvtColor(image, image, cv::COLOR_BGRA2GRAY);
  if (image.type() != CV_8UC1) {
    ++stats_.failures;
    return std::nullopt;
  }
  ++stats_.frames;
  return ImageFrame{
      image, time, time, sequence_++, config_.source_id, TimestampSource::kHostMonotonic};
}
double StereoCalibration::baseline() const {
  return translation.empty() ? 0 : cv::norm(translation);
}
bool StereoCalibration::valid(std::string* reason) const {
  auto fail = [&](const char* s) {
    if (reason)
      *reason = s;
    return false;
  };
  if (image_size.width <= 0 || image_size.height <= 0)
    return fail("invalid image size");
  for (const auto& m : {k_left, k_right, rotation, r_left, r_right})
    if (m.size() != cv::Size(3, 3) || m.type() != CV_64F || !cv::checkRange(m))
      return fail("invalid 3x3 calibration matrix");
  for (const auto& m : {p_left, p_right})
    if (m.size() != cv::Size(4, 3) || m.type() != CV_64F || !cv::checkRange(m))
      return fail("invalid projection matrix");
  if (q.size() != cv::Size(4, 4) || q.type() != CV_64F || !cv::checkRange(q) ||
      translation.total() != 3 || translation.type() != CV_64F || !cv::checkRange(translation))
    return fail("invalid Q or translation");
  for (const auto& m : {d_left, d_right})
    if (m.empty() || m.type() != CV_64F || !cv::checkRange(m))
      return fail("invalid distortion coefficients");
  if (baseline() <= 0 || p_left.at<double>(0, 0) <= 0 || p_left.at<double>(1, 1) <= 0 ||
      std::abs(cv::determinant(rotation) - 1) > 1e-3 || !std::isfinite(rms_error))
    return fail("invalid baseline, rotation, or focal length");
  if (std::abs(q.at<double>(3, 2)) < 1e-12)
    return fail("invalid stereo reprojection geometry");
  return true;
}
StereoCalibration StereoCalibration::load(const std::string& path) {
  if (path.empty()) {
    throw std::invalid_argument("calibration_file is required; supply measured stereo calibration");
  }
  cv::FileStorage f(path, cv::FileStorage::READ);
  if (!f.isOpened())
    throw std::runtime_error("cannot open calibration " + path);
  StereoCalibration c;
  f["K_left"] >> c.k_left;
  f["D_left"] >> c.d_left;
  f["K_right"] >> c.k_right;
  f["D_right"] >> c.d_right;
  f["R"] >> c.rotation;
  f["T"] >> c.translation;
  f["R_left"] >> c.r_left;
  f["R_right"] >> c.r_right;
  f["P_left"] >> c.p_left;
  f["P_right"] >> c.p_right;
  f["Q"] >> c.q;
  c.image_size = {static_cast<int>(f["image_width"]), static_cast<int>(f["image_height"])};
  c.rms_error = static_cast<double>(f["rms_error"]);
  c.provenance = static_cast<std::string>(f["provenance"]);
  std::string reason;
  if (!c.valid(&reason))
    throw std::runtime_error("invalid calibration: " + reason);
  return c;
}
void StereoCalibration::save(const std::string& path) const {
  std::string reason;
  if (!valid(&reason))
    throw std::runtime_error(reason);
  cv::FileStorage f(path, cv::FileStorage::WRITE);
  f << "image_width" << image_size.width << "image_height" << image_size.height << "K_left"
    << k_left << "D_left" << d_left << "K_right" << k_right << "D_right" << d_right << "R"
    << rotation << "T" << translation << "R_left" << r_left << "R_right" << r_right << "P_left"
    << p_left << "P_right" << p_right << "Q" << q << "rms_error" << rms_error << "provenance"
    << provenance;
}
StereoCalibration calibrate_chessboard(const std::vector<cv::Mat>& left,
                                       const std::vector<cv::Mat>& right, cv::Size pattern,
                                       double square, double* usable) {
  if (left.size() != right.size() || left.size() < 3 || pattern.width < 2 || pattern.height < 2 ||
      square <= 0)
    throw std::invalid_argument("invalid chessboard calibration input");
  cv::Size size = left.front().size();
  std::vector<std::vector<cv::Point2f>> lp, rp;
  std::vector<std::vector<cv::Point3f>> objects;
  std::vector<cv::Point3f> object;
  for (int y = 0; y < pattern.height; ++y)
    for (int x = 0; x < pattern.width; ++x)
      object.emplace_back(float(x * square), float(y * square), 0);
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (left[i].size() != size || right[i].size() != size)
      throw std::invalid_argument("calibration image size mismatch");
    cv::Mat l = left[i], r = right[i];
    if (l.channels() > 1)
      cv::cvtColor(l, l, cv::COLOR_BGR2GRAY);
    if (r.channels() > 1)
      cv::cvtColor(r, r, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> a, b;
    if (cv::findChessboardCorners(l, pattern, a) && cv::findChessboardCorners(r, pattern, b)) {
      cv::cornerSubPix(l, a, {5, 5}, {-1, -1},
                       {cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, .01});
      cv::cornerSubPix(r, b, {5, 5}, {-1, -1},
                       {cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, .01});
      lp.push_back(a);
      rp.push_back(b);
      objects.push_back(object);
    }
  }
  if (usable)
    *usable = double(objects.size()) / left.size();
  if (objects.size() < 3)
    throw std::runtime_error("fewer than three usable stereo checkerboard pairs");
  StereoCalibration c;
  c.image_size = size;
  c.k_left = cv::Mat::eye(3, 3, CV_64F);
  c.k_right = cv::Mat::eye(3, 3, CV_64F);
  c.d_left = cv::Mat::zeros(1, 5, CV_64F);
  c.d_right = c.d_left.clone();
  std::vector<cv::Mat> rv, tv;
  cv::calibrateCamera(objects, lp, size, c.k_left, c.d_left, rv, tv);
  cv::calibrateCamera(objects, rp, size, c.k_right, c.d_right, rv, tv);
  cv::Mat essential, fundamental;
  c.rms_error = cv::stereoCalibrate(objects, lp, rp, c.k_left, c.d_left, c.k_right, c.d_right, size,
                                    c.rotation, c.translation, essential, fundamental,
                                    cv::CALIB_FIX_INTRINSIC);
  cv::stereoRectify(c.k_left, c.d_left, c.k_right, c.d_right, size, c.rotation, c.translation,
                    c.r_left, c.r_right, c.p_left, c.p_right, c.q);
  c.provenance = "OpenCV chessboard stereo calibration";
  std::string reason;
  if (!c.valid(&reason))
    throw std::runtime_error("calibration failed validation: " + reason);
  return c;
}
StereoPipeline::StereoPipeline(StereoCalibration c, StereoConfig cfg)
    : calibration_(std::move(c)), config_(cfg) {
  std::string reason;
  if (!calibration_.valid(&reason))
    throw std::invalid_argument(reason);
  if (cfg.num_disparities <= 0 || cfg.num_disparities % 16 || cfg.block_size < 3 ||
      !(cfg.block_size & 1) || cfg.min_depth <= 0 || cfg.max_depth <= cfg.min_depth)
    throw std::invalid_argument("invalid stereo configuration");
  cv::initUndistortRectifyMap(calibration_.k_left, calibration_.d_left, calibration_.r_left,
                              calibration_.p_left, calibration_.image_size, CV_32FC1, lmx_, lmy_);
  cv::initUndistortRectifyMap(calibration_.k_right, calibration_.d_right, calibration_.r_right,
                              calibration_.p_right, calibration_.image_size, CV_32FC1, rmx_, rmy_);
  matcher_ = cv::StereoSGBM::create(
      cfg.min_disparity, cfg.num_disparities, cfg.block_size, 8 * cfg.block_size * cfg.block_size,
      32 * cfg.block_size * cfg.block_size, cfg.disp12_max_diff, 0, cfg.uniqueness_ratio,
      cfg.speckle_window, cfg.speckle_range, cv::StereoSGBM::MODE_SGBM_3WAY);
}
DepthFrame StereoPipeline::process(const cv::Mat& left, const cv::Mat& right) {
  if (left.size() != calibration_.image_size || right.size() != calibration_.image_size ||
      left.empty() || right.empty())
    throw std::invalid_argument("image/calibration size mismatch");
  DepthFrame o;
  auto t = std::chrono::steady_clock::now();
  cv::remap(left, o.rectified_left, lmx_, lmy_, cv::INTER_LINEAR);
  cv::remap(right, o.rectified_right, rmx_, rmy_, cv::INTER_LINEAR);
  if (o.rectified_left.channels() > 1)
    cv::cvtColor(o.rectified_left, o.rectified_left, cv::COLOR_BGR2GRAY);
  if (o.rectified_right.channels() > 1)
    cv::cvtColor(o.rectified_right, o.rectified_right, cv::COLOR_BGR2GRAY);
  o.rectify_ms = ms(t);
  t = std::chrono::steady_clock::now();
  cv::Mat raw;
  matcher_->compute(o.rectified_left, o.rectified_right, raw);
  raw.convertTo(o.disparity, CV_32F, 1. / 16.);
  o.disparity_ms = ms(t);
  t = std::chrono::steady_clock::now();
  o.depth = cv::Mat(o.disparity.size(), CV_32F, std::numeric_limits<float>::quiet_NaN());
  o.valid_mask = cv::Mat::zeros(o.disparity.size(), CV_8U);
  o.confidence = cv::Mat::zeros(o.disparity.size(), CV_32F);
  const double fb = calibration_.p_left.at<double>(0, 0) * calibration_.baseline();
  cv::Mat xyz;
  cv::reprojectImageTo3D(o.disparity, xyz, calibration_.q);
  for (int y = 0; y < o.disparity.rows; ++y)
    for (int x = 0; x < o.disparity.cols; ++x) {
      float d = o.disparity.at<float>(y, x);
      if (d <= 0 || !std::isfinite(d))
        continue;
      float z = xyz.at<cv::Vec3f>(y, x)[2];
      if (!std::isfinite(z) || z < config_.min_depth || z > config_.max_depth)
        continue;
      o.depth.at<float>(y, x) = z;
      o.valid_mask.at<std::uint8_t>(y, x) = 255;
      double sigma_z = std::abs(fb / (d * d)) * config_.disparity_sigma;
      o.confidence.at<float>(y, x) = static_cast<float>(1. / (1. + sigma_z));
    }
  o.depth_ms = ms(t);
  return o;
}
std::vector<cv::Point3f> StereoPipeline::point_cloud(const DepthFrame& d, int stride,
                                                     double* milliseconds) const {
  if (stride < 1)
    throw std::invalid_argument("invalid point-cloud stride");
  auto t = std::chrono::steady_clock::now();
  std::vector<cv::Point3f> out;
  out.reserve(d.depth.total() / (stride * stride));
  double fx = calibration_.p_left.at<double>(0, 0), fy = calibration_.p_left.at<double>(1, 1),
         cx = calibration_.p_left.at<double>(0, 2), cy = calibration_.p_left.at<double>(1, 2);
  for (int y = 0; y < d.depth.rows; y += stride)
    for (int x = 0; x < d.depth.cols; x += stride) {
      float z = d.depth.at<float>(y, x);
      if (std::isfinite(z) && z > 0)
        out.emplace_back(float((x - cx) * z / fx), float((y - cy) * z / fy), z);
    }
  if (milliseconds)
    *milliseconds = ms(t);
  return out;
}
SyntheticStereo::SyntheticStereo(SyntheticConfig c) : config_(c) {
  if (c.size.width <= 0 || c.size.height <= 0 || c.fx <= 0 || c.fy <= 0 || c.baseline <= 0)
    throw std::invalid_argument("invalid synthetic stereo config");
  cv::RNG rng(c.seed);
  texture_ = cv::Mat(c.size, CV_8U);
  rng.fill(texture_, cv::RNG::UNIFORM, 20, 235);
  cv::GaussianBlur(texture_, texture_, {3, 3}, 0);
}
SyntheticFrame SyntheticStereo::render(double stamp, double fg, double bg) const {
  if (fg <= 0 || bg < fg)
    throw std::invalid_argument("invalid synthetic depths");
  SyntheticFrame f;
  f.timestamp = stamp;
  f.left = texture_.clone();
  f.ground_truth_depth = cv::Mat(config_.size, CV_32F, float(bg));
  cv::Rect box(config_.size.width / 3, config_.size.height / 4, config_.size.width / 3,
               config_.size.height / 2);
  if (fg < bg)
    f.ground_truth_depth(box).setTo(float(fg));
  f.right = cv::Mat::zeros(config_.size, CV_8U);
  for (int y = 0; y < config_.size.height; ++y)
    for (int x = 0; x < config_.size.width; ++x) {
      double z = f.ground_truth_depth.at<float>(y, x);
      int xr = int(std::lround(x - config_.fx * config_.baseline / z));
      if (xr >= 0)
        f.right.at<std::uint8_t>(y, xr) = f.left.at<std::uint8_t>(y, x);
    }
  if (config_.noise_std > 0) {
    cv::Mat noise(config_.size, CV_16S);
    cv::RNG rng(config_.seed + static_cast<unsigned>(stamp * 1000));
    rng.fill(noise, cv::RNG::NORMAL, 0, config_.noise_std);
    cv::Mat tmp;
    f.left.convertTo(tmp, CV_16S);
    tmp += noise;
    tmp.convertTo(f.left, CV_8U);
    f.right.convertTo(tmp, CV_16S);
    tmp += noise;
    tmp.convertTo(f.right, CV_8U);
  }
  return f;
}
StereoCalibration SyntheticStereo::calibration() const {
  StereoCalibration c;
  c.k_left =
      (cv::Mat_<double>(3, 3) << config_.fx, 0, config_.cx, 0, config_.fy, config_.cy, 0, 0, 1);
  c.k_right = c.k_left.clone();
  c.d_left = cv::Mat::zeros(1, 5, CV_64F);
  c.d_right = c.d_left.clone();
  c.rotation = cv::Mat::eye(3, 3, CV_64F);
  c.translation = (cv::Mat_<double>(3, 1) << -config_.baseline, 0, 0);
  c.r_left = c.rotation.clone();
  c.r_right = c.rotation.clone();
  c.p_left = (cv::Mat_<double>(3, 4) << config_.fx, 0, config_.cx, 0, 0, config_.fy, config_.cy, 0,
              0, 0, 1, 0);
  c.p_right = c.p_left.clone();
  c.p_right.at<double>(0, 3) = -config_.fx * config_.baseline;
  c.q = (cv::Mat_<double>(4, 4) << 1, 0, 0, -config_.cx, 0, 1, 0, -config_.cy, 0, 0, 0, config_.fx,
         0, 0, 1 / config_.baseline, 0);
  c.image_size = config_.size;
  c.provenance = "synthetic ground truth; not physical calibration";
  return c;
}
void StereoVisualOdometry::reset() {
  previous_image_.release();
  previous_depth_.release();
  previous_stamp_ = 0;
}
OdometryResult StereoVisualOdometry::update(const cv::Mat& image, const cv::Mat& depth,
                                            double stamp) {
  auto begin = std::chrono::steady_clock::now();
  OdometryResult o;
  if (image.empty() || image.type() != CV_8UC1 || depth.type() != CV_32FC1 ||
      depth.size() != image.size() || !std::isfinite(stamp) || stamp <= 0 ||
      (previous_stamp_ && stamp <= previous_stamp_)) {
    o.failure = "invalid image, depth, or timestamp";
    return o;
  }
  auto finish = [&]() {
    image.copyTo(previous_image_);
    depth.copyTo(previous_depth_);
    previous_stamp_ = stamp;
    o.processing_ms = ms(begin);
    return o;
  };
  if (previous_image_.empty()) {
    o.failure = "initializing";
    return finish();
  }
  if (image.size() != previous_image_.size()) {
    o.failure = "image size changed";
    reset();
    return o;
  }
  std::vector<cv::Point2f> a, b, back;
  cv::goodFeaturesToTrack(previous_image_, a, 600, .01, 6);
  o.features = static_cast<int>(a.size());
  if (a.size() < 20) {
    o.failure = "insufficient features";
    return finish();
  }
  std::vector<uchar> status, reverse_status;
  std::vector<float> error, reverse_error;
  cv::calcOpticalFlowPyrLK(previous_image_, image, a, b, status, error, {21, 21}, 3);
  cv::calcOpticalFlowPyrLK(image, previous_image_, b, back, reverse_status, reverse_error, {21, 21},
                           3);
  const double cx = cx_ >= 0 ? cx_ : (image.cols - 1) * .5,
               cy = cy_ >= 0 ? cy_ : (image.rows - 1) * .5;
  cv::Matx33d k(fx_, 0, cx, 0, fy_, cy, 0, 0, 1);
  std::vector<cv::Point3f> object;
  std::vector<cv::Point2f> pixels;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!status[i] || !reverse_status[i] || error[i] > 20 || cv::norm(a[i] - back[i]) > 1)
      continue;
    int x = cvRound(a[i].x), y = cvRound(a[i].y);
    if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows || b[i].x < 1 || b[i].y < 1 ||
        b[i].x >= image.cols - 1 || b[i].y >= image.rows - 1)
      continue;
    float z = previous_depth_.at<float>(y, x);
    if (!std::isfinite(z) || z <= .2 || z > 10)
      continue;
    object.emplace_back((a[i].x - cx) * z / fx_, (a[i].y - cy) * z / fy_, z);
    pixels.push_back(b[i]);
  }
  if (object.size() < 20) {
    o.failure = "insufficient tracked depth features";
    return finish();
  }
  cv::Mat r, t, inliers;
  if (!cv::solvePnPRansac(object, pixels, k, cv::noArray(), r, t, false, 100, 1.5, .999, inliers,
                          cv::SOLVEPNP_EPNP) ||
      inliers.rows < 20 || inliers.rows < int(object.size() * .5)) {
    o.failure = "PnP RANSAC rejected motion";
    return finish();
  }
  std::vector<cv::Point3f> inlier_object;
  std::vector<cv::Point2f> inlier_pixels;
  for (int i = 0; i < inliers.rows; ++i) {
    int j = inliers.at<int>(i);
    inlier_object.push_back(object[j]);
    inlier_pixels.push_back(pixels[j]);
  }
  cv::solvePnPRefineLM(inlier_object, inlier_pixels, k, cv::noArray(), r, t);
  cv::Mat rotation;
  cv::Rodrigues(r, rotation);
  // PnP gives current_from_previous. Invert to obtain camera motion in previous coordinates.
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      o.previous_to_current(i, j) = rotation.at<double>(j, i);
    o.previous_to_current(i, 3) = 0;
    for (int j = 0; j < 3; ++j)
      o.previous_to_current(i, 3) -= rotation.at<double>(j, i) * t.at<double>(j);
  }
  std::vector<cv::Point2f> projected;
  cv::projectPoints(inlier_object, r, t, k, cv::noArray(), projected);
  double squared = 0;
  for (std::size_t i = 0; i < projected.size(); ++i) {
    double e = cv::norm(projected[i] - inlier_pixels[i]);
    squared += e * e;
  }
  o.reprojection_error = std::sqrt(squared / projected.size());
  o.inliers = inliers.rows;
  o.valid = o.reprojection_error < 1.5;
  if (!o.valid)
    o.failure = "excessive reprojection error";
  return finish();
}
}  // namespace ece3162::perception
