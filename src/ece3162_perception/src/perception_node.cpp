#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ece3162_interfaces/msg/vehicle_state.hpp"
#include "ece3162_perception/state_estimator.hpp"
#include "ece3162_perception/stereo_pipeline.hpp"
#include "ece3162_perception/stereo_synchronizer.hpp"
namespace ep = ece3162::perception;
using Clock = std::chrono::steady_clock;
static double ms(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
static geometry_msgs::msg::Quaternion quaternion(const cv::Matx33d& r) {
  cv::Vec3d v;
  cv::Rodrigues(r, v);
  double angle = cv::norm(v);
  geometry_msgs::msg::Quaternion q;
  q.w = 1;
  if (angle > 1e-12) {
    double scale = std::sin(angle * .5) / angle;
    q.x = v[0] * scale;
    q.y = v[1] * scale;
    q.z = v[2] * scale;
    q.w = std::cos(angle * .5);
  }
  return q;
}
class PerceptionNode final : public rclcpp::Node {
 public:
  PerceptionNode() : Node("stereo_perception"), sync_(.005, 4) {
    cv::setNumThreads(1);
    mode_ = declare_parameter("mode", "synthetic");
    double hz = declare_parameter("frame_rate", 10.0);
    declare_parameter("disconnect_after_frames", -1);
    declare_parameter("low_texture_after_frames", -1);
    declare_parameter("drop_every", 0);
    declare_parameter("right_timestamp_offset", 0.0);
    declare_parameter("dynamic_after_s", -1.0);
    declare_parameter("standalone_motion", false);
    declare_parameter("static_obstacle", true);
    declare_parameter("noise_std", 0.0);
    const auto calibration_path = declare_parameter("calibration_file", "");
    const auto recorded = declare_parameter("recorded_manifest", "");
    const auto output = declare_parameter("results_directory", "");
    const auto initial_position =
        declare_parameter("initial_body_position", std::vector<double>{0, 0, 1});
    const auto translation =
        declare_parameter("body_from_left_optical_translation", std::vector<double>{0, 0, 0});
    const auto rotation_vector =
        declare_parameter("body_from_left_optical_rotation_vector", std::vector<double>{0, 0, 0});
    if (initial_position.size() != 3 || translation.size() != 3 || rotation_vector.size() != 3) {
      throw std::invalid_argument("pose parameters must have three SI values");
    }
    for (const auto& values : {initial_position, translation, rotation_vector}) {
      for (double value : values) {
        if (!std::isfinite(value))
          throw std::invalid_argument("non-finite camera/body configuration");
      }
    }
    // Rotation vector is the mounting offset relative to the optical-to-FLU convention.
    body_from_raw_camera_ = ep::camera_pose({translation[0], translation[1], translation[2]});
    cv::Matx33d mounting_rotation;
    cv::Rodrigues(cv::Vec3d(rotation_vector[0], rotation_vector[1], rotation_vector[2]),
                  mounting_rotation);
    cv::Matx44d mount = cv::Matx44d::eye();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        mount(i, j) = mounting_rotation(i, j);
    body_from_raw_camera_ = mount * body_from_raw_camera_;
    // Translation is specified in body coordinates, independent of mounting rotation.
    for (int i = 0; i < 3; ++i) {
      body_from_raw_camera_(i, 3) = translation[i];
    }
    if (!std::isfinite(hz) || hz <= 0 || hz > 100)
      throw std::invalid_argument("invalid frame rate");
    ep::SyntheticConfig sc;
    sc.baseline = .16;
    sc.noise_std = get_parameter("noise_std").as_double();
    synthetic_ = std::make_unique<ep::SyntheticStereo>(sc);
    if (mode_ != "synthetic" && mode_ != "recorded" && mode_ != "external")
      throw std::invalid_argument("mode must be synthetic, recorded, or external");
    auto calibration = mode_ == "synthetic" ? synthetic_->calibration()
                                            : ep::StereoCalibration::load(calibration_path);
    cv::Matx44d raw_from_rectified = cv::Matx44d::eye();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        raw_from_rectified(i, j) = calibration.r_left.at<double>(j, i);
    body_from_camera_ = body_from_raw_camera_ * raw_from_rectified;
    cv::Matx44d initial_body = cv::Matx44d::eye();
    for (int i = 0; i < 3; ++i)
      initial_body(i, 3) = initial_position[i];
    estimator_.reset(initial_body * body_from_camera_);
    render_position_ = {initial_position[0], initial_position[1], initial_position[2]};
    ep::StereoConfig cfg;
    cfg.num_disparities = 64;
    cfg.max_depth = 10;
    pipeline_ = std::make_unique<ep::StereoPipeline>(calibration, cfg);
    vo_ = std::make_unique<ep::StereoVisualOdometry>(
        calibration.p_left.at<double>(0, 0), calibration.p_left.at<double>(1, 1),
        calibration.p_left.at<double>(0, 2), calibration.p_left.at<double>(1, 2));
    if (mode_ == "recorded") {
      manifest_.open(recorded);
      if (!manifest_)
        throw std::runtime_error("cannot open recorded manifest: " + recorded);
      recorded_root_ = std::filesystem::path(recorded).parent_path();
    }
    if (!output.empty()) {
      std::filesystem::create_directories(output);
      output_ = output;
      metrics_.open(output + "/perception.csv");
      metrics_ << "stamp,frame,rectify_ms,disparity_ms,depth_ms,cloud_ms,vo_ms,estimator_ms,total_"
                  "ms,valid_fraction,depth_mae_m,position_error_m,inliers,valid,rejected_pairs\n";
    }
    auto sensor = rclcpp::SensorDataQoS().keep_last(2);
    left_ = create_publisher<sensor_msgs::msg::Image>("stereo/left/image_raw", sensor);
    right_ = create_publisher<sensor_msgs::msg::Image>("stereo/right/image_raw", sensor);
    rect_left_ = create_publisher<sensor_msgs::msg::Image>("stereo/left/image_rect", sensor);
    rect_right_ = create_publisher<sensor_msgs::msg::Image>("stereo/right/image_rect", sensor);
    disparity_ = create_publisher<sensor_msgs::msg::Image>("stereo/disparity", sensor);
    depth_ = create_publisher<sensor_msgs::msg::Image>("stereo/depth", sensor);
    left_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("stereo/left/camera_info", sensor);
    right_info_ =
        create_publisher<sensor_msgs::msg::CameraInfo>("stereo/right/camera_info", sensor);
    cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("perception/occupied_points", sensor);
    odom_ = create_publisher<nav_msgs::msg::Odometry>("visual_odometry", 10);
    state_ = create_publisher<ece3162_interfaces::msg::VehicleState>("perception/state_estimate",
                                                                     sensor);
    diagnostics_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("perception/diagnostics", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publish_static(calibration);
    if (mode_ == "synthetic")
      truth_ = create_subscription<ece3162_interfaces::msg::VehicleState>(
          "state/ground_truth", sensor, [this](ece3162_interfaces::msg::VehicleState::SharedPtr m) {
            render_position_ = {m->position.x, m->position.y, m->position.z};
          });
    if (mode_ == "external") {
      left_sub_ = create_subscription<sensor_msgs::msg::Image>(
          "camera/left/image_raw", sensor,
          [this](sensor_msgs::msg::Image::SharedPtr m) { receive(m, true); });
      right_sub_ = create_subscription<sensor_msgs::msg::Image>(
          "camera/right/image_raw", sensor,
          [this](sensor_msgs::msg::Image::SharedPtr m) { receive(m, false); });
    }
    start_ = now().seconds();
    last_received_ = start_;
    timer_ = create_wall_timer(std::chrono::duration<double>(1 / hz), [this] { tick(); });
    RCLCPP_INFO(
        get_logger(),
        "source=%s; localization=stereo PnP + native state estimator; hardware_output=disabled",
        mode_.c_str());
  }

 private:
  void receive(const sensor_msgs::msg::Image::SharedPtr& m, bool left) {
    double stamp = rclcpp::Time(m->header.stamp).seconds();
    if (now().seconds() - stamp > .25 || stamp > now().seconds() + .01) {
      diagnostic(2, "stale or future image");
      return;
    }
    try {
      enqueue(cv_bridge::toCvCopy(m, "mono8")->image, stamp, left);
    } catch (const std::exception& e) {
      diagnostic(2, e.what());
    }
  }
  void enqueue(const cv::Mat& image, double stamp, bool left) {
    auto& buffer = left ? left_buffer_ : right_buffer_;
    auto seq = sequence_++;
    buffer[seq] = {image, stamp};
    if (left)
      sync_.push_left({seq, stamp});
    else
      sync_.push_right({seq, stamp});
    while (buffer.size() > 4)
      buffer.erase(buffer.begin());
    auto pair = sync_.pop_pair(now().seconds(), .25);
    if (!pair)
      return;
    auto l = left_buffer_.find(pair->left.sequence), r = right_buffer_.find(pair->right.sequence);
    if (l == left_buffer_.end() || r == right_buffer_.end()) {
      diagnostic(2, "missing synchronized image payload");
      return;
    }
    auto li = l->second.first, ri = r->second.first;
    const double time = l->second.second;
    left_buffer_.erase(left_buffer_.begin(), std::next(l));
    right_buffer_.erase(right_buffer_.begin(), std::next(r));
    last_received_ = now().seconds();
    process(li, ri, time);
  }
  void tick() {
    if (mode_ == "external") {
      if (now().seconds() - last_received_ > .5)
        diagnostic(2, "camera image timeout");
      return;
    }
    const int disconnect = get_parameter("disconnect_after_frames").as_int();
    if (disconnect >= 0 && frames_ >= std::size_t(disconnect)) {
      diagnostic(2, "camera disconnected");
      return;
    }
    auto stamp = now();
    cv::Mat left, right;
    if (mode_ == "recorded") {
      if (!recorded_pending_) {
        if (!(manifest_ >> pending_recorded_stamp_ >> pending_left_path_ >> pending_right_path_)) {
          diagnostic(2, "recorded sequence exhausted");
          return;
        }
        if (!std::isfinite(pending_recorded_stamp_) || pending_recorded_stamp_ < 0 ||
            (recorded_initialized_ && pending_recorded_stamp_ <= last_recorded_stamp_)) {
          throw std::runtime_error("invalid or non-monotonic recorded timestamp");
        }
        if (!recorded_initialized_) {
          recorded_epoch_ = pending_recorded_stamp_;
          replay_epoch_ = now().seconds();
          recorded_initialized_ = true;
        }
        recorded_pending_ = true;
      }
      const double observation_time = replay_epoch_ + pending_recorded_stamp_ - recorded_epoch_;
      if (now().seconds() < observation_time)
        return;
      stamp = rclcpp::Time(static_cast<int64_t>(observation_time * 1e9));
      last_recorded_stamp_ = pending_recorded_stamp_;
      recorded_pending_ = false;
      left = cv::imread((recorded_root_ / pending_left_path_).string(), cv::IMREAD_GRAYSCALE);
      right = cv::imread((recorded_root_ / pending_right_path_).string(), cv::IMREAD_GRAYSCALE);
      if (left.empty() || right.empty())
        throw std::runtime_error("missing recorded stereo image");
      gt_depth_.release();
    } else {
      if (get_parameter("standalone_motion").as_bool()) {
        double t = stamp.seconds() - start_;
        render_position_ = {.2 * std::sin(.3 * t), .1 * std::sin(.4 * t),
                            1 + .05 * std::sin(.2 * t)};
      }
      std::vector<ep::SceneBox> boxes;
      if (get_parameter("static_obstacle").as_bool())
        boxes.push_back({{2.5, -.4, 0}, {2.9, .4, 2.0}});
      double dynamic = get_parameter("dynamic_after_s").as_double();
      if (dynamic >= 0 && stamp.seconds() - start_ >= dynamic)
        boxes.push_back({{3.3, -2.0, .6}, {3.55, 2.0, 1.5}});
      int low = get_parameter("low_texture_after_frames").as_int();
      cv::Matx44d render_body = cv::Matx44d::eye();
      for (int i = 0; i < 3; ++i)
        render_body(i, 3) = render_position_[i];
      auto frame = synthetic_->render_pose(stamp.seconds(), render_body * body_from_raw_camera_,
                                           boxes, low >= 0 && frames_ >= std::size_t(low));
      left = frame.left;
      right = frame.right;
      gt_depth_ = frame.ground_truth_depth;
    }
    ++frames_;
    int drop = get_parameter("drop_every").as_int();
    if (drop > 0 && frames_ % drop == 0) {
      diagnostic(1, "configured stereo frame drop");
      return;
    }
    publish_image(left, "mono8", stamp, "left_camera_optical_frame", left_);
    publish_image(right, "mono8", stamp, "right_camera_optical_frame", right_);
    enqueue(left, stamp.seconds(), true);
    enqueue(right, stamp.seconds() + get_parameter("right_timestamp_offset").as_double(), false);
  }
  void process(const cv::Mat& left, const cv::Mat& right, double time) {
    auto begin = Clock::now();
    auto d = pipeline_->process(left, right);
    auto result = vo_->update(d.rectified_left, d.depth, time);
    auto estimation_begin = Clock::now();
    auto state = estimator_.update(result, time);
    double estimator_ms = ms(estimation_begin);
    rclcpp::Time stamp(static_cast<int64_t>(time * 1e9));
    bool valid = state.usable(now().seconds());
    publish_image(d.rectified_left, "mono8", stamp, "left_camera_rectified_frame", rect_left_);
    publish_image(d.rectified_right, "mono8", stamp, "right_camera_rectified_frame", rect_right_);
    publish_image(d.disparity, "32FC1", stamp, "left_camera_rectified_frame", disparity_);
    publish_image(d.depth, "32FC1", stamp, "left_camera_rectified_frame", depth_);
    publish_info(stamp);
    publish_state(state, stamp, valid);
    double cloud_ms{};
    auto points = pipeline_->point_cloud(d, 4, &cloud_ms);
    if (valid)
      publish_cloud(points, state.world_from_camera, stamp);
    double valid_fraction = cv::countNonZero(d.valid_mask) / double(d.valid_mask.total()), mae = 0,
           pose_error = 0;
    if (!gt_depth_.empty()) {
      cv::Mat diff;
      cv::absdiff(d.depth, gt_depth_, diff);
      mae = cv::mean(diff, d.valid_mask)[0];
      cv::Vec3d estimated(state.world_from_camera(0, 3), state.world_from_camera(1, 3),
                          state.world_from_camera(2, 3));
      pose_error = cv::norm(estimated - render_position_);
    }
    if (metrics_) {
      metrics_ << std::setprecision(15) << time << ',' << frames_ << ',' << d.rectify_ms << ','
               << d.disparity_ms << ',' << d.depth_ms << ',' << cloud_ms << ','
               << result.processing_ms << ',' << estimator_ms << ',' << ms(begin) << ','
               << valid_fraction << ',' << mae << ',' << pose_error << ',' << result.inliers << ','
               << valid << ',' << sync_.stats().rejected << '\n';
      metrics_.flush();
    }
    if (!output_.empty() && !saved_) {
      cv::imwrite(output_ + "/left.png", left);
      cv::imwrite(output_ + "/right.png", right);
      cv::Mat disp;
      d.disparity.convertTo(disp, CV_8U, 4);
      cv::imwrite(output_ + "/disparity.png", disp);
      saved_ = true;
    }
    diagnostic(valid ? 0 : 1, valid                    ? "tracking"
                              : result.failure.empty() ? "localization invalid; reset required"
                                                       : result.failure);
  }
  template <class P>
  void publish_image(const cv::Mat& m, const std::string& encoding, const rclcpp::Time& s,
                     const std::string& frame, const P& p) {
    std_msgs::msg::Header h;
    h.stamp = s;
    h.frame_id = frame;
    p->publish(*cv_bridge::CvImage(h, encoding, m).toImageMsg());
  }
  void publish_info(const rclcpp::Time& s) {
    const auto& c = pipeline_->calibration();
    for (int eye = 0; eye < 2; ++eye) {
      sensor_msgs::msg::CameraInfo m;
      m.header.stamp = s;
      m.header.frame_id = eye ? "right_camera_optical_frame" : "left_camera_optical_frame";
      m.width = c.image_size.width;
      m.height = c.image_size.height;
      m.distortion_model = "plumb_bob";
      const cv::Mat &k = eye ? c.k_right : c.k_left, &r = eye ? c.r_right : c.r_left,
                    &p = eye ? c.p_right : c.p_left, &d = eye ? c.d_right : c.d_left;
      std::copy_n(k.ptr<double>(), 9, m.k.begin());
      std::copy_n(r.ptr<double>(), 9, m.r.begin());
      std::copy_n(p.ptr<double>(), 12, m.p.begin());
      m.d.assign(d.ptr<double>(), d.ptr<double>() + d.total());
      (eye ? right_info_ : left_info_)->publish(m);
    }
  }
  void publish_cloud(const std::vector<cv::Point3f>& points, const cv::Matx44d& pose,
                     const rclcpp::Time& s) {
    sensor_msgs::msg::PointCloud2 m;
    m.header.stamp = s;
    m.header.frame_id = "map";
    sensor_msgs::PointCloud2Modifier mod(m);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(m, "x"), y(m, "y"), z(m, "z");
    for (const auto& p : points) {
      auto q = pose * cv::Vec4d(p.x, p.y, p.z, 1);
      *x = q[0];
      *y = q[1];
      *z = q[2];
      ++x;
      ++y;
      ++z;
    }
    cloud_->publish(m);
  }
  void publish_state(const ep::EstimatedState& s, const rclcpp::Time& stamp, bool valid) {
    const cv::Matx44d world_from_body = s.world_from_camera * body_from_camera_.inv();
    const cv::Vec3d body_position(world_from_body(0, 3), world_from_body(1, 3),
                                  world_from_body(2, 3));
    if (previous_body_stamp_ > 0 && stamp.seconds() > previous_body_stamp_ && valid) {
      const double dt = stamp.seconds() - previous_body_stamp_;
      const double alpha = dt / (.15 + dt);
      body_velocity_ =
          (1 - alpha) * body_velocity_ + alpha * (body_position - previous_body_position_) / dt;
    }
    previous_body_position_ = body_position;
    previous_body_stamp_ = stamp.seconds();
    ece3162_interfaces::msg::VehicleState m;
    m.header.stamp = stamp;
    m.header.frame_id = "map";
    m.position.x = body_position[0];
    m.position.y = body_position[1];
    m.position.z = body_position[2];
    m.velocity.x = body_velocity_[0];
    m.velocity.y = body_velocity_[1];
    m.velocity.z = body_velocity_[2];
    m.valid = valid;
    m.ground_truth = false;
    state_->publish(m);
    cv::Matx33d rotation;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        rotation(i, j) = world_from_body(i, j);
      }
    nav_msgs::msg::Odometry o;
    o.header = m.header;
    o.child_frame_id = "base_link";
    o.pose.pose.position = m.position;
    o.pose.pose.orientation = quaternion(rotation);
    const auto local_velocity = rotation.t() * body_velocity_;
    o.twist.twist.linear.x = local_velocity[0];
    o.twist.twist.linear.y = local_velocity[1];
    o.twist.twist.linear.z = local_velocity[2];
    for (int i = 0; i < 6; ++i)
      o.pose.covariance[7 * i] = s.covariance(i, i);
    odom_->publish(o);
    geometry_msgs::msg::TransformStamped t;
    t.header = m.header;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = m.position.x;
    t.transform.translation.y = m.position.y;
    t.transform.translation.z = m.position.z;
    t.transform.rotation = o.pose.pose.orientation;
    tf_->sendTransform(t);
  }
  void publish_static(const ep::StereoCalibration& calibration) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    auto add = [&](std::string parent, std::string child, cv::Vec3d p, cv::Matx33d r) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = parent;
      t.child_frame_id = child;
      t.transform.translation.x = p[0];
      t.transform.translation.y = p[1];
      t.transform.translation.z = p[2];
      t.transform.rotation = quaternion(r);
      transforms.push_back(t);
    };
    auto identity = cv::Matx33d::eye();
    add("map", "odom", {}, identity);
    cv::Matx33d optical(0, 0, 1, -1, 0, 0, 0, -1, 0);
    cv::Matx44d right_from_left = cv::Matx44d::eye();
    for (int i = 0; i < 3; ++i) {
      right_from_left(i, 3) = calibration.translation.at<double>(i);
      for (int j = 0; j < 3; ++j)
        right_from_left(i, j) = calibration.rotation.at<double>(i, j);
    }
    const cv::Matx44d right_pose = body_from_raw_camera_ * right_from_left.inv();
    for (int eye = 0; eye < 2; ++eye) {
      const auto pose = eye ? right_pose : body_from_raw_camera_;
      cv::Matx33d rotation, rectified;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
          rotation(i, j) = pose(i, j);
          rectified(i, j) = (eye ? calibration.r_right : calibration.r_left).at<double>(j, i);
        }
      const std::string side = eye ? "right" : "left";
      add("base_link", side + "_camera_link", {pose(0, 3), pose(1, 3), pose(2, 3)},
          rotation * optical.t());
      add(side + "_camera_optical_frame", side + "_camera_rectified_frame", {}, rectified);
    }
    add("left_camera_link", "left_camera_optical_frame", {}, optical);
    add("right_camera_link", "right_camera_optical_frame", {}, optical);
    static_tf_->sendTransform(transforms);
  }
  void diagnostic(int level, const std::string& message) {
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "stereo_perception";
    s.hardware_id = mode_;
    s.level = level;
    s.message = message;
    diagnostic_msgs::msg::KeyValue v;
    v.key = "source";
    v.value = mode_ + "; stereo PnP localization; no ground-truth estimator input";
    s.values.push_back(v);
    v.key = "rejected_pairs";
    v.value = std::to_string(sync_.stats().rejected);
    s.values.push_back(v);
    a.status.push_back(s);
    diagnostics_->publish(a);
  }
  std::string mode_, output_;
  std::unique_ptr<ep::SyntheticStereo> synthetic_;
  std::unique_ptr<ep::StereoPipeline> pipeline_;
  std::unique_ptr<ep::StereoVisualOdometry> vo_;
  ep::StateEstimator estimator_;
  ep::StereoSynchronizer sync_;
  std::map<std::uint64_t, std::pair<cv::Mat, double>> left_buffer_, right_buffer_;
  std::uint64_t sequence_{};
  std::size_t frames_{};
  bool saved_{};
  double start_{}, last_received_{}, last_recorded_stamp_{};
  double pending_recorded_stamp_{}, recorded_epoch_{}, replay_epoch_{};
  bool recorded_pending_{}, recorded_initialized_{};
  std::filesystem::path recorded_root_;
  std::string pending_left_path_, pending_right_path_;
  cv::Vec3d render_position_{0, 0, 1};
  cv::Matx44d body_from_camera_{cv::Matx44d::eye()}, body_from_raw_camera_{cv::Matx44d::eye()};
  cv::Vec3d previous_body_position_{}, body_velocity_{};
  double previous_body_stamp_{};
  cv::Mat gt_depth_;
  std::ifstream manifest_;
  std::ofstream metrics_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_, right_, rect_left_, rect_right_,
      disparity_, depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_, right_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<ece3162_interfaces::msg::VehicleState>::SharedPtr state_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<ece3162_interfaces::msg::VehicleState>::SharedPtr truth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_, right_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PerceptionNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("stereo_perception"), "%s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
}
