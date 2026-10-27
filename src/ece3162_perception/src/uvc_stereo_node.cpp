#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ece3162_perception/stereo_pipeline.hpp"
namespace ep = ece3162::perception;
// Acquisition only. Shared perception_node synchronizes payloads and processes all sources.
class UvcStereoNode final : public rclcpp::Node {
 public:
  UvcStereoNode() : Node("uvc_stereo") {
    auto left = declare_parameter("left_device", "/dev/v4l/by-id/LEFT_OV9281_UNCONFIGURED");
    auto right = declare_parameter("right_device", "/dev/v4l/by-id/RIGHT_OV9281_UNCONFIGURED");
    auto file = declare_parameter("calibration_file", "");
    int width = declare_parameter("width", 640), height = declare_parameter("height", 480);
    double fps = declare_parameter("fps", 20.), exposure = declare_parameter("exposure", -1.),
           gain = declare_parameter("gain", -1.);
    auto format = declare_parameter("pixel_format", "");
    if (width < 16 || height < 16 || width > 4096 || height > 4096 || !std::isfinite(fps) ||
        fps <= 0 || fps > 240 || left == right)
      throw std::invalid_argument("invalid dual-camera configuration");
    auto calibration = ep::StereoCalibration::load(file);
    if (calibration.image_size != cv::Size(width, height))
      throw std::invalid_argument("camera resolution/calibration mismatch");
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
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("perception/diagnostics", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / fps), [this] { tick(); });
  }

 private:
  void publish(const ep::ImageFrame& f, bool left) {
    // Translate host monotonic reception time into ROS time without labelling it exposure time.
    const double mono =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    std_msgs::msg::Header h;
    h.stamp = now() - rclcpp::Duration::from_seconds(std::max(0., mono - f.receive_time));
    h.frame_id = left ? "left_camera_optical_frame" : "right_camera_optical_frame";
    (left ? left_pub_ : right_pub_)->publish(*cv_bridge::CvImage(h, "mono8", f.image).toImageMsg());
  }
  void tick() {
    auto l = left_cam_->read();
    if (l)
      publish(*l, true);
    auto r = right_cam_->read();
    if (r)
      publish(*r, false);
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "uvc_acquisition";
    s.hardware_id = "configured-dual-uvc";
    s.level = l && r ? 0 : 2;
    s.message = l && r ? "frames received" : "camera acquisition failure/disconnect";
    auto add = [&](std::string k, std::string v) {
      diagnostic_msgs::msg::KeyValue x;
      x.key = k;
      x.value = v;
      s.values.push_back(x);
    };
    add("timestamp_provenance",
        "host monotonic receipt mapped to ROS; exposure synchronization unverified");
    add("left_sequence", std::to_string(left_cam_->stats().frames));
    add("right_sequence", std::to_string(right_cam_->stats().frames));
    add("driver_dropped_frames", "unavailable through OpenCV; receipt-gap estimate only");
    add("estimated_receipt_drops",
        std::to_string(left_cam_->stats().dropped + right_cam_->stats().dropped));
    a.status.push_back(s);
    diag_->publish(a);
  }
  std::unique_ptr<ep::UvcCamera> left_cam_, right_cam_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_, right_pub_;
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
