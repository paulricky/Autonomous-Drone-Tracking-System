#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iterator>
#include <atomic>
#include <mutex>
#include <thread>
#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "stereo_perception/stereo_pipeline.hpp"
namespace ep = robotics::perception;
// Acquisition only. Shared perception_node synchronizes payloads and processes all sources.
class UvcStereoNode final : public rclcpp::Node {
 public:
  UvcStereoNode() : Node("uvc_stereo") {
    auto left = declare_parameter("left_device", "/dev/v4l/by-id/LEFT_OV9281_UNCONFIGURED");
    auto right = declare_parameter("right_device", "/dev/v4l/by-id/RIGHT_OV9281_UNCONFIGURED");
    auto file = declare_parameter("calibration_file", "");
    const auto timestamp_policy = declare_parameter("timestamp_policy", "host_monotonic_receipt_ros_mapped");
    left_frame_ = declare_parameter("left_frame_id", "left_camera_optical_frame");
    right_frame_ = declare_parameter("right_frame_id", "right_camera_optical_frame");
    int width = declare_parameter("width", 640), height = declare_parameter("height", 480);
    double fps = declare_parameter("fps", 20.), exposure = declare_parameter("exposure", -1.),
           gain = declare_parameter("gain", -1.);
    auto format = declare_parameter("pixel_format", "");
    if (left.find("UNCONFIGURED") != std::string::npos ||
        right.find("UNCONFIGURED") != std::string::npos)
      throw std::invalid_argument("measured persistent left and right camera paths are required");
    if (!std::filesystem::exists(left) || !std::filesystem::exists(right))
      throw std::invalid_argument("one or both configured camera device paths are unavailable");
    if (width < 16 || height < 16 || width > 4096 || height > 4096 || !std::isfinite(fps) ||
        fps <= 0 || fps > 240 || left == right ||
        left_frame_ != "left_camera_optical_frame" ||
        right_frame_ != "right_camera_optical_frame" ||
        timestamp_policy != "host_monotonic_receipt_ros_mapped" ||
        (left.rfind("/dev/v4l/by-id/", 0) != 0 && left.rfind("/dev/v4l/by-path/", 0) != 0) ||
        (right.rfind("/dev/v4l/by-id/", 0) != 0 && right.rfind("/dev/v4l/by-path/", 0) != 0))
      throw std::invalid_argument("invalid dual-camera configuration");
    if (std::filesystem::equivalent(left, right))
      throw std::invalid_argument("left and right camera paths resolve to the same video device");
    auto calibration = ep::StereoCalibration::load(file);
    if (calibration.provenance != "OpenCV chessboard stereo calibration" ||
        calibration.rms_error > 1.0 || calibration.left_rms_error > 1.0 ||
        calibration.right_rms_error > 1.0)
      throw std::invalid_argument("physical operation requires measured, quality-checked stereo calibration");
    if (calibration.left_camera_id.empty() || calibration.left_camera_id != left ||
        calibration.right_camera_id != right)
      throw std::invalid_argument("camera device identities do not match measured calibration");
    if (calibration.image_size != cv::Size(width, height))
      throw std::invalid_argument("camera resolution/calibration mismatch");
    calibration_ = std::move(calibration);
    int fourcc = 0;
    if (!format.empty()) {
      if (format.size() != 4)
        throw std::invalid_argument("pixel_format must have four characters");
      fourcc = cv::VideoWriter::fourcc(format[0], format[1], format[2], format[3]);
    }
    ep::CameraConfig l{left, "left", width, height, fps, exposure, gain, fourcc};
    auto r = l;
    r.device = right;
    r.source_id = "right";
    left_cam_ = std::make_unique<ep::UvcCamera>(l);
    right_cam_ = std::make_unique<ep::UvcCamera>(r);
    for (const auto& control : left_cam_->unsupported_controls())
      RCLCPP_WARN(get_logger(), "left camera unsupported control: %s", control.c_str());
    for (const auto& control : right_cam_->unsupported_controls())
      RCLCPP_WARN(get_logger(), "right camera unsupported control: %s", control.c_str());
    left_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw",
                                                          rclcpp::SensorDataQoS().keep_last(1));
    right_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw",
                                                           rclcpp::SensorDataQoS().keep_last(1));
    left_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/left/camera_info", rclcpp::SensorDataQoS().keep_last(1));
    right_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/right/camera_info", rclcpp::SensorDataQoS().keep_last(1));
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("perception/diagnostics", 10);
    running_ = true;
    left_thread_ = std::thread([this] { acquire(true); });
    right_thread_ = std::thread([this] { acquire(false); });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { tick(); });
  }
  ~UvcStereoNode() override {
    running_ = false;
    if (left_thread_.joinable()) left_thread_.join();
    if (right_thread_.joinable()) right_thread_.join();
  }

 private:
  void publish(const ep::ImageFrame& f, bool left) {
    // Translate host monotonic reception time into ROS time without labelling it exposure time.
    std_msgs::msg::Header h;
    const auto ros_anchor = now();
    const double monotonic_anchor = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    h.stamp = ros_anchor - rclcpp::Duration::from_seconds(std::max(0., monotonic_anchor - f.receive_time));
    h.frame_id = left ? left_frame_ : right_frame_;
    (left ? left_pub_ : right_pub_)->publish(*cv_bridge::CvImage(h, "mono8", f.image).toImageMsg());
    sensor_msgs::msg::CameraInfo info;
    info.header = h;
    info.width = calibration_.image_size.width;
    info.height = calibration_.image_size.height;
    info.distortion_model = "plumb_bob";
    const auto& k = left ? calibration_.k_left : calibration_.k_right;
    const auto& d = left ? calibration_.d_left : calibration_.d_right;
    std::copy_n(k.ptr<double>(), 9, info.k.begin());
    std::copy_n(d.ptr<double>(), d.total(), std::back_inserter(info.d));
    // CameraInfo carries raw K/D and the measured rectification R/P, so
    // downstream image processing can rectify the corresponding raw image.
    const auto& r = left ? calibration_.r_left : calibration_.r_right;
    const auto& p = left ? calibration_.p_left : calibration_.p_right;
    std::copy_n(r.ptr<double>(), 9, info.r.begin());
    std::copy_n(p.ptr<double>(), 12, info.p.begin());
    (left ? left_info_ : right_info_)->publish(info);
  }
  void acquire(bool left) {
    auto& camera = left ? left_cam_ : right_cam_;
    while (running_) {
      const auto frame = camera->read();
      if (frame) publish(*frame, left);
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        (left ? left_stats_ : right_stats_) = camera->stats();
      }
      if (!frame) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  void tick() {
    ep::AcquisitionStats l, r;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      l = left_stats_;
      r = right_stats_;
    }
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "uvc_acquisition";
    s.hardware_id = "configured-dual-uvc";
    s.level = l.frames > last_left_frames_ && r.frames > last_right_frames_ ? 0 : 2;
    s.message = s.level == 0 ? "frames received" : "camera acquisition failure/disconnect";
    last_left_frames_ = l.frames;
    last_right_frames_ = r.frames;
    auto add = [&](std::string k, std::string v) {
      diagnostic_msgs::msg::KeyValue x;
      x.key = k;
      x.value = v;
      s.values.push_back(x);
    };
    add("timestamp_provenance",
        "host monotonic receipt mapped to ROS; exposure synchronization unverified");
    add("left_sequence", std::to_string(l.frames));
    add("right_sequence", std::to_string(r.frames));
    add("driver_dropped_frames", "unavailable through OpenCV; receipt-gap estimate only");
    add("estimated_receipt_drops",
        std::to_string(l.dropped + r.dropped));
    add("reconnects", std::to_string(l.reconnects + r.reconnects));
    add("read_failures", std::to_string(l.failures + r.failures));
    a.status.push_back(s);
    diag_->publish(a);
  }
  std::unique_ptr<ep::UvcCamera> left_cam_, right_cam_;
  ep::StereoCalibration calibration_;
  std::string left_frame_, right_frame_;
  std::atomic<bool> running_{false};
  std::thread left_thread_, right_thread_;
  std::mutex stats_mutex_;
  ep::AcquisitionStats left_stats_, right_stats_;
  std::size_t last_left_frames_{}, last_right_frames_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_, right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_, right_info_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UvcStereoNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("uvc_stereo"), "%s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
}
