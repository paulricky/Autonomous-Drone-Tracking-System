#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include "ece3162_core/bspline.hpp"
#include "ece3162_core/dynamics.hpp"
#include "ece3162_core/ego_optimizer.hpp"
#include "ece3162_core/kinodynamic_astar.hpp"
#include "ece3162_core/trajectory_tracker.hpp"
#include "ece3162_interfaces/action/navigate_to_pose.hpp"
#include "ece3162_interfaces/msg/control_command.hpp"
#include "ece3162_interfaces/msg/planner_status.hpp"
#include "ece3162_interfaces/msg/timed_trajectory.hpp"
#include "ece3162_interfaces/msg/vehicle_state.hpp"

using namespace std::chrono_literals;
namespace nav = ece3162::navigation;
static nav::Vec3 vec(const geometry_msgs::msg::Vector3& v) {
  return {v.x, v.y, v.z};
}
static nav::Vec3 point(const geometry_msgs::msg::Point& v) {
  return {v.x, v.y, v.z};
}
static void set(geometry_msgs::msg::Vector3& v, const nav::Vec3& a) {
  v.x = a[0];
  v.y = a[1];
  v.z = a[2];
}
static void set(geometry_msgs::msg::Point& v, const nav::Vec3& a) {
  v.x = a[0];
  v.y = a[1];
  v.z = a[2];
}

class SimulatorNode final : public rclcpp::Node {
 public:
  SimulatorNode()
      : Node("simulated_drone"),
        sim_({{{0, 0, 1}}, {{0, 0, 0}}, 0}, {.02, 1.5, 2, .002, .003, .04, 7}) {
    RCLCPP_INFO(get_logger(), "simulated drone initialized; hardware output absent");
    command_ = create_subscription<ece3162_interfaces::msg::ControlCommand>(
        "control/acceleration", rclcpp::QoS(1).best_effort(),
        [this](ece3162_interfaces::msg::ControlCommand::SharedPtr m) {
          std::lock_guard<std::mutex> l(mu_);
          acceleration_ = vec(m->acceleration);
          braking_ = m->braking;
          last_command_ = now();
        });
    truth_ = create_publisher<ece3162_interfaces::msg::VehicleState>("state/ground_truth", 10);
    estimate_ = create_publisher<ece3162_interfaces::msg::VehicleState>("state/estimate",
                                                                        rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(20ms, [this] { tick(); });
  }

 private:
  void tick() {
    nav::Vec3 a{};
    {
      std::lock_guard<std::mutex> l(mu_);
      if ((now() - last_command_).seconds() < .15 && !braking_)
        a = acceleration_;
      else {
        const auto v = sim_.ground_truth().velocity;
        const double speed = nav::norm(v);
        if (speed > 1e-6)
          a = nav::mul(v, -std::min(2., speed / .02) / speed);
      }
    }
    const auto& gt = sim_.step(a);
    const auto est = sim_.estimated_state();
    publish(gt, true, truth_);
    publish(est, false, estimate_);
  }
  void publish(const nav::PlanningState& s, bool gt,
               const rclcpp::Publisher<ece3162_interfaces::msg::VehicleState>::SharedPtr& p) {
    ece3162_interfaces::msg::VehicleState m;
    m.header.stamp = now();
    m.header.frame_id = "map";
    set(m.position, s.position);
    set(m.velocity, s.velocity);
    m.valid = true;
    m.ground_truth = gt;
    p->publish(m);
  }
  nav::DoubleIntegrator sim_;
  nav::Vec3 acceleration_{};
  bool braking_{};
  std::mutex mu_;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<ece3162_interfaces::msg::ControlCommand>::SharedPtr command_;
  rclcpp::Publisher<ece3162_interfaces::msg::VehicleState>::SharedPtr truth_, estimate_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class AutonomyNode final : public rclcpp::Node {
  using Navigate = ece3162_interfaces::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Navigate>;

 public:
  AutonomyNode()
      : Node("autonomy_pipeline"),
        map_({-1, -2, 0}, {7, 2, 3}, .25, nav::Occupancy::kFree),
        tracker_({4, 2.8, 2, 2, .25}) {
    RCLCPP_INFO(get_logger(), "initializing native autonomy pipeline");
    declare_parameter("scenario", "static");
    declare_parameter("goal_x", 5.0);
    declare_parameter("goal_y", 0.0);
    declare_parameter("goal_z", 1.0);
    declare_parameter("auto_start", true);
    declare_parameter("state_topic", "state/estimate");
    declare_parameter("use_perception_map", false);
    declare_parameter("minimum_map_updates", 3);
    declare_parameter("unknown_is_free", false);
    declare_parameter("state_timeout", .25);
    tracker_ = ece3162::control::TrajectoryTracker(
        {4, 2.8, 2, 2, get_parameter("state_timeout").as_double()});
    declare_parameter("hardware_output_enabled", false);
    if (get_parameter("hardware_output_enabled").as_bool())
      throw std::runtime_error("physical hardware output is forbidden in simulation");
    scenario_ = get_parameter("scenario").as_string();
    use_perception_map_ = get_parameter("use_perception_map").as_bool();
    goal_ = {get_parameter("goal_x").as_double(), get_parameter("goal_y").as_double(),
             get_parameter("goal_z").as_double()};
    attempted_ = !get_parameter("auto_start").as_bool();
    if (use_perception_map_) {
      if (!get_parameter("unknown_is_free").as_bool())
        map_.fill(nav::Occupancy::kUnknown);
    } else
      configure_map();
    occupancy_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("mapping/occupied_voxels",
                                                                     rclcpp::SensorDataQoS());
    command_ = create_publisher<ece3162_interfaces::msg::ControlCommand>(
        "control/acceleration", rclcpp::QoS(1).best_effort());
    trajectory_pub_ = create_publisher<ece3162_interfaces::msg::TimedTrajectory>(
        "planning/trajectory", rclcpp::QoS(1).transient_local().reliable());
    status_ = create_publisher<std_msgs::msg::String>("mission/status", 10);
    planner_status_ =
        create_publisher<ece3162_interfaces::msg::PlannerStatus>("planning/status", 10);
    estimate_ = create_subscription<ece3162_interfaces::msg::VehicleState>(
        get_parameter("state_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](ece3162_interfaces::msg::VehicleState::SharedPtr m) { on_state(*m); });
    if (use_perception_map_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "perception/occupied_points", rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { on_cloud(*m); });
    }
    action_ = rclcpp_action::create_server<Navigate>(
        this, "navigate_to_pose",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Navigate::Goal> g) {
          const auto& p = g->goal.pose.position;
          return (g->goal.header.frame_id.empty() || g->goal.header.frame_id == "map") &&
                         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)
                     ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                     : rclcpp_action::GoalResponse::REJECT;
        },
        [this](const std::shared_ptr<GoalHandle>) {
          cancel_requested_ = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> h) {
          const auto& p = h->get_goal()->goal.pose.position;
          goal_ = {p.x, p.y, p.z};
          active_goal_ = h;
          cancel_requested_ = false;
          trajectory_.reset();
          attempted_ = false;
        });
    timer_ = create_wall_timer(20ms, [this] { control(); });
  }

 private:
  void configure_map() {
    if (scenario_ == "static") {
      map_.fill_box({1.5, -.6, 0}, {2.5, .6, 2.5});
      map_.fill_box({3.2, -2, 0}, {3.8, -.4, 2.5});
    } else if (scenario_ == "constrained") {
      map_.fill_box({1, -2, 0}, {1.7, .3, 2.4});
      map_.fill_box({2.4, -.3, 0}, {3.1, 2, 2.4});
    } else if (scenario_ == "failure")
      map_.fill_box({1, -2, 0}, {4.5, 2, 3});
  }
  void on_state(const ece3162_interfaces::msg::VehicleState& m) {
    std::lock_guard<std::mutex> l(mu_);
    auto position = point(m.position), velocity = vec(m.velocity);
    double stamp = rclcpp::Time(m.header.stamp).seconds();
    if (m.header.frame_id != "map" || m.ground_truth || !nav::finite(position) ||
        !nav::finite(velocity) || stamp <= state_.stamp || stamp > now().seconds() + .01) {
      state_valid_ = false;
      return;
    }
    state_ = {position, velocity, stamp};
    state_valid_ = m.valid;
  }
  void on_cloud(const sensor_msgs::msg::PointCloud2& m) {
    if (m.header.frame_id != "map" || now().seconds() - rclcpp::Time(m.header.stamp).seconds() > .5)
      return;
    nav::PlanningState pose;
    {
      std::lock_guard<std::mutex> l(mu_);
      if (!state_valid_)
        return;
      pose = state_;
    }
    if (std::abs(pose.stamp - rclcpp::Time(m.header.stamp).seconds()) > .15)
      return;
    nav::Vec3 sensor_origin;
    try {
      const auto transform = tf_buffer_->lookupTransform("map", "left_camera_rectified_frame",
                                                         rclcpp::Time(m.header.stamp),
                                                         rclcpp::Duration::from_seconds(.02));
      sensor_origin = vec(transform.transform.translation);
    } catch (const tf2::TransformException&) {
      return;  // No ray clearing without a timestamp-aligned sensor origin.
    }
    std::lock_guard<std::mutex> map_lock(map_mu_);
    const auto begin = std::chrono::steady_clock::now();
    try {
      std::vector<nav::Vec3> points;
      sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        nav::Vec3 p{*x, *y, *z};
        if (nav::finite(p))
          points.push_back(p);
      }
      if (points.empty())
        return;
      map_.integrate_cloud(sensor_origin, points, 8);
      ++map_updates_;
      map_changed_ = true;
      last_map_stamp_ = rclcpp::Time(m.header.stamp).seconds();
      last_map_ms_ =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
              .count();
      sensor_msgs::msg::PointCloud2 occupied;
      occupied.header = m.header;
      sensor_msgs::PointCloud2Modifier mod(occupied);
      mod.setPointCloud2FieldsByString(1, "xyz");
      auto centers = map_.occupied_centers();
      mod.resize(centers.size());
      sensor_msgs::PointCloud2Iterator<float> ox(occupied, "x"), oy(occupied, "y"),
          oz(occupied, "z");
      for (const auto& p : centers) {
        *ox = p[0];
        *oy = p[1];
        *oz = p[2];
        ++ox;
        ++oy;
        ++oz;
      }
      occupancy_pub_->publish(occupied);
    } catch (const std::runtime_error&) {
      map_corrupt_ = true;
    }
  }
  void publish_status(const std::string& s) {
    if (s != last_status_) {
      RCLCPP_INFO(get_logger(), "mission_status=%s", s.c_str());
      last_status_ = s;
    }
    std_msgs::msg::String m;
    m.data = s;
    status_->publish(m);
  }
  bool plan(const nav::PlanningState& start) {
    publish_status("planning");
    ece3162_interfaces::msg::PlannerStatus ps;
    ps.stamp = now();
    ps.state = ps.PLANNING;
    planner_status_->publish(ps);
    const auto pipeline_begin = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> map_lock(map_mu_);
    // Plan with additional reserve for later stereo observations; active safety
    // validation retains the 0.25 m physical envelope and never ignores collisions.
    nav::VoxelMap collision_map = use_perception_map_ ? map_.inflated(.5) : map_;
    RCLCPP_INFO(get_logger(),
                "planning state=(%.3f,%.3f,%.3f) velocity=(%.3f,%.3f,%.3f) occupied=%zu",
                start.position[0], start.position[1], start.position[2], start.velocity[0],
                start.velocity[1], start.velocity[2], collision_map.occupied_centers().size());
    nav::KinodynamicConfig cfg;
    cfg.heuristic_weight = 2;
    cfg.timeout_s = 5;
    auto global = nav::KinodynamicAStar(cfg).plan(start, goal_, collision_map);
    last_planning_ms_ = global.planning_time_s * 1000;
    ps.planning_latency_ms = last_planning_ms_;
    ps.explored_states = global.expanded;
    if (!global.success) {
      ps.state = ps.FAILED;
      ps.detail = global.message;
      planner_status_->publish(ps);
      publish_status("failed:" + global.message);
      finish_action(false, global.message);
      return false;
    }
    std::vector<nav::Vec3> guide;
    for (auto& s : global.states)
      guide.push_back(s.position);
    if (nav::norm(nav::sub(guide.back(), goal_)) > .01)
      guide.push_back(goal_);
    while (guide.size() < 4)
      guide.insert(guide.end() - 1, nav::mul(nav::add(guide[guide.size() - 2], guide.back()), .5));
    auto local = nav::EgoOptimizer().optimize(guide, collision_map, std::max(5., global.cost * 1.8),
                                              {1.5, 2});
    last_optimization_ms_ = local.optimization_time_s * 1000;
    if (!local.success && use_perception_map_) {
      // The unconstrained smoothness cost can cut a global route's corners.
      // Retry with stronger route adherence, always validating the actual curve.
      for (double guidance_weight : {20.0, 200.0}) {
        nav::EgoConfig recovery;
        recovery.guidance_weight = guidance_weight;
        recovery.collision_weight = 1000.0;
        auto candidate = nav::EgoOptimizer(recovery).optimize(
            guide, collision_map, std::max(5., global.cost * 1.8), {1.5, 2});
        last_optimization_ms_ += candidate.optimization_time_s * 1000;
        local = std::move(candidate);
        if (local.success)
          break;
      }
    }
    if (!local.success) {
      ps.state = ps.FAILED;
      ps.detail = local.message;
      planner_status_->publish(ps);
      publish_status("failed:" + local.message);
      finish_action(false, local.message);
      return false;
    }
    trajectory_ = local.trajectory;
    trajectory_start_ = now();
    ++trajectory_id_;
    publish_trajectory();
    last_pipeline_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pipeline_begin)
            .count();
    ps.state = ps.TRACKING;
    ps.detail = "native global and local trajectory accepted";
    planner_status_->publish(ps);
    publish_status(replans_ ? "tracking:replanned" : "tracking");
    return true;
  }
  void publish_trajectory() {
    ece3162_interfaces::msg::TimedTrajectory msg;
    msg.header.stamp = trajectory_start_;
    msg.header.frame_id = "map";
    msg.trajectory_id = trajectory_id_;
    msg.duration = trajectory_->duration();
    for (auto& s : trajectory_->sample(.02)) {
      ece3162_interfaces::msg::TrajectoryPoint p;
      p.time_from_start = rclcpp::Duration::from_seconds(s.time);
      set(p.position, s.position);
      set(p.velocity, s.velocity);
      set(p.acceleration, s.acceleration);
      msg.points.push_back(p);
    }
    trajectory_pub_->publish(msg);
  }
  void control() {
    nav::PlanningState s;
    bool valid;
    {
      std::lock_guard<std::mutex> l(mu_);
      s = state_;
      valid = state_valid_;
    }
    if (cancel_requested_ && active_goal_) {
      brake();
      publish_status("cancelled:operator request");
      auto result = std::make_shared<Navigate::Result>();
      result->success = false;
      result->detail = "operator cancellation";
      active_goal_->canceled(result);
      active_goal_.reset();
      trajectory_.reset();
      cancel_requested_ = false;
      attempted_ = true;
      return;
    }
    if (map_corrupt_) {
      publish_status("failed:map corruption");
      brake();
      rclcpp::shutdown();
      return;
    }
    if (use_perception_map_ &&
        map_updates_ < std::size_t(get_parameter("minimum_map_updates").as_int())) {
      publish_status("waiting:perception map");
      return;
    }
    if (!valid || now().seconds() - s.stamp > .5 ||
        (use_perception_map_ && last_map_stamp_ > 0 && now().seconds() - last_map_stamp_ > 1.0)) {
      if (trajectory_) {
        brake();
        publish_status("failed:localization or map unavailable");
        finish_action(false, "localization or map unavailable");
        rclcpp::shutdown();
      } else
        publish_status("waiting:localization");
      return;
    }
    if (!trajectory_) {
      if (!attempted_) {
        attempted_ = true;
        if (!plan(s) && !active_goal_)
          rclcpp::shutdown();
      }
      return;
    }
    double elapsed = (now() - trajectory_start_).seconds();
    if (use_perception_map_ && map_changed_) {
      map_changed_ = false;
      if (active_unsafe(elapsed)) {
        brake();
        replan_event_begin_ = std::chrono::steady_clock::now();
        awaiting_replan_control_ = true;
        trajectory_.reset();
        ++replans_;
        if (!plan(s)) {
          brake();
          rclcpp::shutdown();
          return;
        }
        elapsed = 0;
      }
    }
    if (scenario_ == "dynamic" && !dynamic_added_ && elapsed > 2) {
      dynamic_added_ = true;
      replan_event_begin_ = std::chrono::steady_clock::now();
      awaiting_replan_control_ = true;
      map_.fill_box({2.3, -.45, 0}, {3.0, .45, 2.5});
      if (active_unsafe(elapsed)) {
        trajectory_.reset();
        ++replans_;
        if (!plan(s)) {
          brake();
          if (!active_goal_)
            rclcpp::shutdown();
          return;
        }
        elapsed = 0;
      } else
        awaiting_replan_control_ = false;
    }
    if (!trajectory_)
      return;
    const auto callback_begin = std::chrono::steady_clock::now();
    if (last_control_ != std::chrono::steady_clock::time_point{})
      control_intervals_ms_.push_back(
          std::chrono::duration<double, std::milli>(callback_begin - last_control_).count());
    last_control_ = callback_begin;
    double t = std::clamp(elapsed, 0., trajectory_->duration());
    auto r = trajectory_->evaluate(t);
    if (t >= trajectory_->duration()) {
      r.velocity = {};
      r.acceleration = {};
    }
    ece3162::control::Reference ref{r.position, r.velocity, r.acceleration,
                                    trajectory_start_.seconds() + t};
    ece3162::control::State st{s.position, s.velocity, s.stamp, valid};
    const double control_now = now().seconds();
    auto c = tracker_.update(control_now, ref, st);
    if (!c) {
      RCLCPP_ERROR(get_logger(), "tracker rejected state: reason=%s age_s=%.6f state_valid=%d",
                   tracker_.failure_reason().c_str(), control_now - st.stamp, valid);
      publish_status("failed:" + tracker_.failure_reason());
      finish_action(false, tracker_.failure_reason());
      brake();
      if (!active_goal_)
        rclcpp::shutdown();
      return;
    }
    tracking_error_squared_ += c->tracking_error * c->tracking_error;
    max_tracking_error_ = std::max(max_tracking_error_, c->tracking_error);
    ++tracking_samples_;
    ece3162_interfaces::msg::ControlCommand msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.trajectory_id = trajectory_id_;
    set(msg.acceleration, c->acceleration);
    command_->publish(msg);
    if (awaiting_replan_control_) {
      RCLCPP_INFO(get_logger(), "perception_replan image_to_control_ms=%.6f map_update_ms=%.6f",
                  (now().seconds() - last_map_stamp_) * 1000, last_map_ms_);
      integrated_replan_ms_ = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - replan_event_begin_)
                                  .count();
      awaiting_replan_control_ = false;
    }
    callback_times_ms_.push_back(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - callback_begin)
            .count());
    if (active_goal_) {
      auto feedback = std::make_shared<Navigate::Feedback>();
      feedback->current_pose.header.stamp = now();
      feedback->current_pose.header.frame_id = "map";
      set(feedback->current_pose.pose.position, s.position);
      feedback->distance_remaining = nav::norm(nav::sub(s.position, goal_));
      feedback->planner_state = ece3162_interfaces::msg::PlannerStatus::TRACKING;
      active_goal_->publish_feedback(feedback);
    }
    if (elapsed >= trajectory_->duration() && nav::norm(nav::sub(s.position, goal_)) < .15 &&
        nav::norm(s.velocity) < .15) {
      publish_status("succeeded");
      report_metrics(s);
      finish_action(true, "goal reached");
      trajectory_.reset();
      attempted_ = true;
      if (!active_goal_)
        rclcpp::shutdown();
    }
  }
  void finish_action(bool success, const std::string& detail) {
    if (!active_goal_)
      return;
    auto handle = active_goal_;
    active_goal_.reset();
    auto result = std::make_shared<Navigate::Result>();
    result->success = success;
    result->detail = detail;
    if (success)
      handle->succeed(result);
    else
      handle->abort(result);
  }
  void report_metrics(const nav::PlanningState& s) {
    auto mean = [](const std::vector<double>& v) {
      return v.empty() ? 0. : std::accumulate(v.begin(), v.end(), 0.) / v.size();
    };
    auto maximum = [](const std::vector<double>& v) {
      return v.empty() ? 0. : *std::max_element(v.begin(), v.end());
    };
    std::size_t misses = std::count_if(control_intervals_ms_.begin(), control_intervals_ms_.end(),
                                       [](double x) { return x > 20.0; });
    RCLCPP_INFO(
        get_logger(),
        "metrics planning_ms=%.6f optimization_ms=%.6f pipeline_ms=%.6f integrated_replan_ms=%.6f "
        "replans=%zu rms_error_m=%.6f max_error_m=%.6f final_error_m=%.6f control_mean_ms=%.6f "
        "control_max_ms=%.6f deadline_misses=%zu callback_mean_ms=%.6f callback_max_ms=%.6f",
        last_planning_ms_, last_optimization_ms_, last_pipeline_ms_, integrated_replan_ms_,
        replans_, std::sqrt(tracking_error_squared_ / std::max<std::size_t>(1, tracking_samples_)),
        max_tracking_error_, nav::norm(nav::sub(s.position, goal_)), mean(control_intervals_ms_),
        maximum(control_intervals_ms_), misses, mean(callback_times_ms_),
        maximum(callback_times_ms_));
  }
  builtin_interfaces::msg::Time state_stamp() {
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(state_.stamp);
    t.nanosec = static_cast<uint32_t>((state_.stamp - t.sec) * 1e9);
    return t;
  }
  bool active_unsafe(double elapsed) {
    std::lock_guard<std::mutex> map_lock(map_mu_);
    auto collision_map = use_perception_map_ ? map_.inflated(.25) : map_;
    for (double t = std::max(0., elapsed); t <= trajectory_->duration(); t += .03)
      if (collision_map.occupied(trajectory_->position(t)))
        return true;
    return false;
  }
  void brake() {
    ece3162_interfaces::msg::ControlCommand m;
    m.header.stamp = now();
    m.braking = true;
    nav::Vec3 a{};
    {
      std::lock_guard<std::mutex> l(mu_);
      double n = nav::norm(state_.velocity);
      if (n > 1e-6)
        a = nav::mul(state_.velocity, -std::min(2.0, n / .02) / n);
    }
    set(m.acceleration, a);
    command_->publish(m);
  }
  nav::VoxelMap map_;
  nav::Vec3 goal_;
  std::string scenario_, last_status_;
  ece3162::control::TrajectoryTracker tracker_;
  std::shared_ptr<nav::BSpline> trajectory_;
  nav::PlanningState state_{};
  bool state_valid_{}, attempted_{}, dynamic_added_{}, awaiting_replan_control_{},
      cancel_requested_{}, use_perception_map_{}, map_changed_{}, map_corrupt_{};
  std::size_t replans_{}, tracking_samples_{}, map_updates_{};
  std::uint64_t trajectory_id_{};
  double last_map_stamp_{}, last_map_ms_{}, last_planning_ms_{}, last_optimization_ms_{},
      last_pipeline_ms_{}, integrated_replan_ms_{}, tracking_error_squared_{},
      max_tracking_error_{};
  std::vector<double> control_intervals_ms_, callback_times_ms_;
  std::chrono::steady_clock::time_point last_control_{}, replan_event_begin_{};
  std::mutex mu_, map_mu_;
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  rclcpp_action::Server<Navigate>::SharedPtr action_;
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp::Publisher<ece3162_interfaces::msg::ControlCommand>::SharedPtr command_;
  rclcpp::Publisher<ece3162_interfaces::msg::TimedTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<ece3162_interfaces::msg::PlannerStatus>::SharedPtr planner_status_;
  rclcpp::Subscription<ece3162_interfaces::msg::VehicleState>::SharedPtr estimate_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto simulator = std::make_shared<SimulatorNode>();
  auto autonomy = std::make_shared<AutonomyNode>();
  exec->add_node(simulator);
  exec->add_node(autonomy);
  exec->spin();
  rclcpp::shutdown();
}
