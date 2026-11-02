// PX4 stabilized acceleration interface through MAVROS. No raw RC/motor output.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "flight_interfaces/msg/control_command.hpp"
#include "flight_interfaces/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/param_pull.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/vehicle_info_get.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotics_core/controller_interface.hpp"
#include "robotics_core/operator_session.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
class FlightAdapter : public rclcpp::Node, public robotics::flight::ControllerBackend {
  using Target = mavros_msgs::msg::PositionTarget;
  using Mode = mavros_msgs::srv::SetMode;
  using Arm = mavros_msgs::srv::CommandBool;

 public:
  FlightAdapter() : Node("physical_flight_adapter") {
    uid_ = declare_parameter("controller_uid", std::string("UNCONFIGURED"));
    version_ = declare_parameter("firmware_version", int64_t(0));
    rc_channel_ = declare_parameter("pilot_enable_channel", -1);
    rc_min_ = declare_parameter("pilot_enable_min", 0);
    rc_max_ = declare_parameter("pilot_enable_max", 0);
    takeoff_modes_ = declare_parameter("valid_takeoff_modes", std::vector<std::string>{});
    if (takeoff_modes_.empty())
      throw std::runtime_error("commissioned valid takeoff modes required");
    max_acc_ = declare_parameter("max_acceleration_mps2", 0.0);
    param_names_ = declare_parameter("verified_parameter_names", std::vector<std::string>{});
    param_values_ = declare_parameter("verified_parameter_values", std::vector<double>{});
    if (uid_ == "UNCONFIGURED" || version_ <= 0 || rc_channel_ < 0 || rc_min_ <= 0 ||
        rc_max_ <= rc_min_ || !std::isfinite(max_acc_) || max_acc_ <= 0 ||
        param_names_.size() != param_values_.size() || param_names_.empty())
      throw std::runtime_error(
          "commissioned PX4 identity, pilot channel, limits and failsafe parameters required");
    for (double value : param_values_)
      if (!std::isfinite(value))
        throw std::runtime_error("invalid controller parameter snapshot");
    yaw_ = 0;
    active_ = create_publisher<std_msgs::msg::Bool>("flight/navigation_active",
                                                    rclcpp::QoS(1).transient_local().reliable());
    emergency_sub_ = create_subscription<std_msgs::msg::Empty>(
        "flight/emergency_request", rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::Empty::SharedPtr) {
          request(robotics::flight::ControllerOperation::Emergency);
        });
    authorization_sub_ = create_subscription<std_msgs::msg::Bool>(
        "mission/operator_authorized", 1, [this](std_msgs::msg::Bool::SharedPtr m) {
          if (m->data && !authorized_ && !land_latched_)
            navigation_fault_ = false;
          authorized_ = m->data;
          authorization_time_ = steady();
        });
    covariance_ = declare_parameter("vision_pose_covariance_diagonal", std::vector<double>{});
    if (covariance_.size() != 6)
      throw std::runtime_error("measured VO covariance required");
    for (double v : covariance_)
      if (!std::isfinite(v) || v <= 0)
        throw std::runtime_error("invalid VO covariance");
    vision_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "mavros/vision_pose/pose_cov", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "visual_odometry", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
          double age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
          if (!vo_valid_ || steady() - vo_time_ > .25 ||
              std::abs(rclcpp::Time(m->header.stamp).seconds() - vo_stamp_) > .1 || age < 0 ||
              age > .25 || m->header.frame_id != "map")
            return;
          const auto q = m->pose.pose.orientation;
          double qn = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
          if (!std::isfinite(qn) || std::abs(qn - 1) > .01)
            return;
          vo_orientation_ = q;
          orientation_time_ = steady();
          const auto l = local_.pose.orientation;
          double dot = std::abs(q.x * l.x + q.y * l.y + q.z * l.z + q.w * l.w);
          orientation_agrees_ = steady() - local_time_ < .25 && std::isfinite(dot) &&
                                dot > std::cos(.1) && dot < 1.001;
          geometry_msgs::msg::PoseWithCovarianceStamped out;
          out.header = m->header;
          out.pose = m->pose;
          out.pose.covariance.fill(0);
          for (size_t i = 0; i < 6; ++i)
            out.pose.covariance[i * 7] = covariance_[i];
          vision_->publish(out);
        });
    out_ = create_publisher<Target>("mavros/setpoint_raw/local", rclcpp::SensorDataQoS());
    status_ = create_publisher<std_msgs::msg::String>("flight/status", 10);
    mode_ = create_client<Mode>("mavros/set_mode");
    arm_ = create_client<Arm>("mavros/cmd/arming");
    info_ = create_client<mavros_msgs::srv::VehicleInfoGet>("mavros/vehicle_info_get");
    param_ = create_client<rcl_interfaces::srv::GetParameters>("mavros/param/get_parameters");
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, [this](mavros_msgs::msg::State::SharedPtr m) {
          auto expected = expected_exits_.find(m->mode);
          const bool requested_exit =
              expected != expected_exits_.end() && steady() < expected->second;
          if (offboard_seen_ && state_.mode == "OFFBOARD" && m->mode != "OFFBOARD" &&
              !requested_exit)
            externally_armed_ =
                true;  // Pilot/controller mode change latches autonomous output off.
          if (requested_exit)
            expected_exits_.erase(expected);
          state_ = *m;
          state_time_ = steady();
          if (state_.mode == "OFFBOARD") {
            offboard_seen_ = true;
          }
          if (state_.mode == expected_mode_) {
            expected_mode_.clear();
            mode_exit_authorized_ = false;
          }
          if (state_.armed && !last_armed_)
            externally_armed_ = externally_armed_ || !arm_requested_;
          last_armed_ = state_.armed;
        });
    ext_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
        "mavros/extended_state", 10, [this](mavros_msgs::msg::ExtendedState::SharedPtr m) {
          ext_ = *m;
          ext_time_ = steady();
        });
    rc_sub_ = create_subscription<mavros_msgs::msg::RCIn>(
        "mavros/rc/in", rclcpp::SensorDataQoS(), [this](mavros_msgs::msg::RCIn::SharedPtr m) {
          rc_time_ = steady();
          bool was_pilot = pilot_;
          pilot_ = size_t(rc_channel_) < m->channels.size() &&
                   m->channels[rc_channel_] >= rc_min_ && m->channels[rc_channel_] <= rc_max_;
          if (was_pilot && !pilot_ && state_.armed && (arm_requested_ || takeoff_requested_)) {
            externally_armed_ = true;
            permit_ = false;
            detail_ = "pilot authorization withdrawn; autonomy latched off";
          }
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "mavros/imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::SharedPtr m) {
          double age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
          auto q = m->orientation;
          double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
          bool finite =
              std::isfinite(n) && std::abs(n - 1) < .01 && std::isfinite(m->angular_velocity.x) &&
              std::isfinite(m->angular_velocity.y) && std::isfinite(m->angular_velocity.z) &&
              std::isfinite(m->linear_acceleration.x) && std::isfinite(m->linear_acceleration.y) &&
              std::isfinite(m->linear_acceleration.z);
          if (age >= 0 && age < .25 && finite && m->orientation_covariance[0] >= 0)
            imu_time_ = steady();
        });
    local_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          auto age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
          auto p = m->pose.position;
          auto q = m->pose.orientation;
          double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
          if (age < 0 || age > .25 || m->header.frame_id != "map" || !std::isfinite(p.x) ||
              !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(n) ||
              std::abs(n - 1) > .01)
            return;
          local_ = *m;
          local_time_ = steady();
          yaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        });
    estimate_sub_ = create_subscription<flight_interfaces::msg::VehicleState>(
        "perception/state_estimate", rclcpp::SensorDataQoS(),
        [this](flight_interfaces::msg::VehicleState::SharedPtr m) {
          auto age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
          localized_ =
              m->valid && !m->ground_truth && m->header.frame_id == "map" && age >= 0 && age < .25;
          localized_ = localized_ && std::isfinite(m->position.x) && std::isfinite(m->position.y) &&
                       std::isfinite(m->position.z) && std::isfinite(m->velocity.x) &&
                       std::isfinite(m->velocity.y) && std::isfinite(m->velocity.z);
          vo_valid_ = localized_;
          vo_time_ = steady();
          vo_stamp_ = rclcpp::Time(m->header.stamp).seconds();
          double dx = m->position.x - local_.pose.position.x,
                 dy = m->position.y - local_.pose.position.y,
                 dz = m->position.z - local_.pose.position.z;
          localized_ = localized_ && steady() - local_time_ < .25 &&
                       std::sqrt(dx * dx + dy * dy + dz * dz) < .2;
          estimate_time_ = steady();
        });
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "perception/occupied_points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {
          auto age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
          if (m->width * m->height > 0 && m->header.frame_id == "map" && age >= 0 && age < .25)
            map_time_ = steady();
        });
    permit_sub_ = create_subscription<std_msgs::msg::Bool>(
        "mission/navigation_enabled", 1, [this](std_msgs::msg::Bool::SharedPtr m) {
          permit_ = m->data;
          permit_time_ = steady();
        });
    op_sub_ = create_subscription<std_msgs::msg::String>(
        "flight/operator_request", rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::String::SharedPtr m) { operation(m->data); });
    control_sub_ = create_subscription<flight_interfaces::msg::ControlCommand>(
        "control/acceleration", rclcpp::SensorDataQoS(),
        [this](flight_interfaces::msg::ControlCommand::SharedPtr m) {
          robotics::flight::MotionSetpoint sample{
              rclcpp::Time(m->header.stamp).seconds(),
              {m->acceleration.x, m->acceleration.y, m->acceleration.z},
              m->braking,
              m->header.frame_id};
          if (submit_motion(sample))
            command_ = *m;
        });
    timer_ = create_wall_timer(20ms, [this] { tick(); });
    verify_ = create_wall_timer(2s, [this] { verify(); });
  }

  robotics::flight::ControllerCapabilities capabilities() const override {
    bool verified = identity_ && parameters_;
    return {identity_, verified, verified, verified, verified,
            verified,  verified, verified, verified};
  }
  robotics::flight::ControllerFeedback feedback() const override {
    std::string phase = "fault";
    if (connected()) {
      if (emergency_)
        phase = "emergency_recovery";
      else if (ground())
        phase = state_.armed ? "armed" : (ready() ? "hardware_ready" : "disarmed");
      else if (state_.mode == "AUTO.LAND")
        phase = "landing";
      else if (state_.mode == "AUTO.TAKEOFF")
        phase = "takeoff_in_progress";
      else if (ext_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR)
        phase = state_.mode == "OFFBOARD" ? "navigating" : "hovering";
    }
    return {
        connected(),
        state_.armed,
        ground(),
        connected() && ext_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR,
        externally_armed_,
        state_.mode,
        phase,
        detail_};
  }
  void request(robotics::flight::ControllerOperation op) override {
    using Op = robotics::flight::ControllerOperation;
    switch (op) {
      case Op::Arm:
        operation("ARM");
        break;
      case Op::Takeoff:
        operation("TAKEOFF");
        break;
      case Op::Hold:
        operation("HOLD");
        break;
      case Op::Land:
        operation("LAND");
        break;
      case Op::Emergency:
        operation("EMERGENCY");
        break;
      case Op::Disarm:
        operation("DISARM");
        break;
    }
  }
  bool submit_motion(const robotics::flight::MotionSetpoint& sample) override {
    double age = now().seconds() - sample.stamp;
    double n = 0;
    for (double a : sample.acceleration_enu) {
      if (!std::isfinite(a))
        return false;
      n += a * a;
    }
    if (age < 0 || age > .15 || sample.frame != "map" || std::sqrt(n) > max_acc_ ||
        sample.stamp <= command_stamp_)
      return false;
    command_stamp_ = sample.stamp;
    command_time_ = steady();
    return true;
  }

 private:
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  bool connected() const {
    return state_.connected && steady() - state_time_ < 2.5 && steady() - ext_time_ < 2.5;
  }
  bool ready() const {
    return connected() && identity_ && parameters_ && pilot_ && steady() - rc_time_ < .3 &&
           localized_ && steady() - estimate_time_ < .25 && steady() - map_time_ < .3 &&
           steady() - imu_time_ < .25 && steady() - orientation_time_ < .25 &&
           orientation_agrees_ && !emergency_ && !land_latched_ && !externally_armed_;
  }
  std::string blocked() const {
    std::string reasons;
    auto add = [&reasons](bool ok, const char* reason) {
      if (!ok) {
        if (!reasons.empty())
          reasons += "; ";
        reasons += reason;
      }
    };
    add(connected(), "controller telemetry unavailable");
    add(identity_, "PX4 UID/firmware identity unverified");
    add(parameters_, "commissioned controller parameters not verified");
    add(pilot_ && steady() - rc_time_ < .3, "independent pilot authorization unavailable");
    add(localized_ && steady() - estimate_time_ < .25, "VO/EKF position invalid or stale");
    add(orientation_agrees_, "VO/EKF heading disagreement");
    add(steady() - imu_time_ < .25, "controller IMU unavailable");
    add(steady() - map_time_ < .3, "stereo depth unavailable");
    add(!emergency_, "emergency latched");
    add(!land_latched_, "landing latched");
    add(authorized_ && steady() - authorization_time_ < .3, "operator authorization unavailable");
    add(!externally_armed_, "pilot/controller takeover latched");
    return reasons;
  }
  bool ground() const {
    return connected() &&
           ext_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
  }
  void set_mode(const std::string& name) {
    if (externally_armed_ || !pilot_ || steady() - rc_time_ > .3) {
      detail_ = "REJECTED mode change: independent pilot recovery has authority";
      return;
    }
    expected_mode_ = name;
    if (name != "OFFBOARD")
      expected_exits_[name] = steady() + 3;
    mode_exit_authorized_ = name != "OFFBOARD";
    if (!mode_->service_is_ready()) {
      expected_exits_.erase(name);
      expected_mode_.clear();
      mode_exit_authorized_ = false;
      detail_ = "REJECTED mode service unavailable";
      return;
    }
    auto r = std::make_shared<Mode::Request>();
    r->custom_mode = name;
    mode_->async_send_request(r, [this, name](rclcpp::Client<Mode>::SharedFuture f) {
      if (!f.get()->mode_sent) {
        expected_exits_.erase(name);
        if (expected_mode_ == name) {
          expected_mode_.clear();
          mode_exit_authorized_ = false;
        }
      }
      detail_ = f.get()->mode_sent ? "mode request sent; awaiting telemetry: " + name
                                   : "REJECTED mode: " + name;
    });
  }
  void arm(bool value) {
    if (!arm_->service_is_ready()) {
      detail_ = "REJECTED arming service unavailable";
      return;
    }
    auto r = std::make_shared<Arm::Request>();
    r->value = value;
    arm_->async_send_request(r, [this](rclcpp::Client<Arm>::SharedFuture f) {
      detail_ =
          f.get()->success ? "controller ACK; awaiting armed feedback" : "REJECTED by controller";
    });
  }
  void operation(const std::string& op) {
    if (op == "EMERGENCY") {
      if (emergency_)
        return;
      emergency_ = true;
      land_latched_ = true;
      permit_ = false;
      detail_ = "emergency latched";
      if (connected() && identity_ && parameters_ && !externally_armed_ && state_.armed && pilot_ &&
          steady() - rc_time_ < .3)
        set_mode("AUTO.LAND");
      return;
    }
    if (op == "LAND") {
      land_latched_ = true;
      permit_ = false;
      navigation_fault_ = true;
      if (connected() && identity_ && parameters_ && !externally_armed_ && state_.armed)
        set_mode("AUTO.LAND");
      else
        detail_ = "REJECTED LAND: verified controller unavailable";
      return;
    }
    if (op == "HOLD") {
      permit_ = false;
      navigation_fault_ = true;
      if (connected() && identity_ && parameters_ && !externally_armed_ && state_.armed &&
          !land_latched_)
        set_mode("AUTO.LOITER");
      return;
    }
    if (op == "DISARM") {
      if (ground() && identity_ && parameters_)
        arm(false);
      else
        detail_ = "REJECTED disarm: ground not confirmed";
      return;
    }
    if (op == "ARM" && authorized_ && steady() - authorization_time_ < .3 && ready() && ground() &&
        !state_.armed && !arm_requested_ && !land_latched_) {
      arm_requested_ = true;
      arm(true);
      return;
    }
    if (op == "TAKEOFF" && authorized_ && steady() - authorization_time_ < .3 &&
        std::find(takeoff_modes_.begin(), takeoff_modes_.end(), state_.mode) !=
            takeoff_modes_.end() &&
        ready() && ground() && state_.armed && arm_requested_ && !takeoff_requested_ &&
        !land_latched_) {
      takeoff_requested_ = true;
      takeoff_start_ = steady();
      set_mode("AUTO.TAKEOFF");
      return;
    }
    detail_ = "REJECTED invalid transition or readiness interlock: " + op;
  }
  void verify() {
    if (!state_.connected || !info_->service_is_ready() || !param_->service_is_ready()) {
      identity_ = parameters_ = false;
      return;
    }
    auto r = std::make_shared<mavros_msgs::srv::VehicleInfoGet::Request>();
    r->get_all = false;  // GET_MY_SYSID/COMPID: verify the controller MAVROS actually commands.
    info_->async_send_request(
        r, [this](rclcpp::Client<mavros_msgs::srv::VehicleInfoGet>::SharedFuture f) {
          identity_time_ = steady();
          identity_ = false;
          for (auto& v : f.get()->vehicles)
            if (v.autopilot == 12 && v.type == 2 && std::to_string(v.uid) == uid_ &&
                v.flight_sw_version == version_ && (v.available_info & 2) &&
                (v.capabilities & 128ULL))
              identity_ = true;
        });
    // Only one parameter query in flight, never reinterpret a firmware default as commissioning.
    if (param_pending_)
      return;
    auto p = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    p->names = param_names_;
    param_pending_ = true;
    param_->async_send_request(
        p, [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture f) {
          auto r = f.get();
          parameters_time_ = steady();
          param_pending_ = false;
          parameters_ = r->values.size() == param_values_.size();
          for (size_t i = 0; i < r->values.size() && parameters_; ++i) {
            auto v = r->values[i];
            double actual = v.type == 2 ? double(v.integer_value) : v.double_value;
            if ((v.type != 2 && v.type != 3) || !std::isfinite(actual) ||
                std::abs(actual - param_values_[i]) > 1e-5)
              parameters_ = false;
          }
          if (!parameters_)
            detail_ = "BLOCKED controller failsafe/control parameter mismatch";
        });
  }
  void tick() {
    auto t = steady();
    if (steady() - identity_time_ > 5)
      identity_ = false;
    if (steady() - parameters_time_ > 5)
      parameters_ = false;
    if (land_latched_ && connected() && identity_ && parameters_ && state_.armed && pilot_ &&
        t - rc_time_ < .3 && !externally_armed_ && state_.mode != "AUTO.LAND" &&
        t - last_mode_request_ > 1) {
      set_mode("AUTO.LAND");
      last_mode_request_ = t;
    }
    bool permitted = permit_ && t - permit_time_ < .3;
    if (state_.armed && (arm_requested_ || takeoff_requested_) && !externally_armed_ &&
        (!connected() || !identity_ || !parameters_ || !pilot_ || t - rc_time_ > .3 ||
         !localized_ || t - estimate_time_ > .25 || t - map_time_ > .3 || t - permit_time_ > .5)) {
      if (!land_latched_)
        operation("EMERGENCY");
    }
    if (takeoff_requested_ && ground() && t - takeoff_start_ > 5 && state_.mode != "AUTO.TAKEOFF" &&
        !land_latched_) {
      detail_ = "takeoff mode acknowledgement timeout; landing recovery latched";
      operation("LAND");
    }
    bool nav = ready() && state_.armed && takeoff_requested_ && permitted &&
               (state_.mode == "AUTO.LOITER" || state_.mode == "OFFBOARD") && !navigation_fault_ &&
               !land_latched_ &&
               ext_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
    bool active = nav && state_.mode == "OFFBOARD";
    std_msgs::msg::Bool active_msg;
    active_msg.data = active;
    active_->publish(active_msg);
    if (nav) {
      if (stream_started_ == 0) {
        stream_started_ = t;
        offboard_start_ = 0;
        hold_yaw_ = yaw_;
        command_time_ = -1e9;
        command_.braking = false;
      }
      Target out;
      out.header.stamp = now();
      out.header.frame_id = "map";
      out.coordinate_frame = Target::FRAME_LOCAL_NED;
      out.yaw = hold_yaw_;
      bool tracking = active && t - command_time_ < .15 && !command_.braking;
      if (tracking) {
        out.type_mask = Target::IGNORE_PX | Target::IGNORE_PY | Target::IGNORE_PZ |
                        Target::IGNORE_VX | Target::IGNORE_VY | Target::IGNORE_VZ |
                        Target::IGNORE_YAW_RATE;
        out.acceleration_or_force = command_.acceleration;
        had_control_ = true;
      } else {
        // Velocity zero primes OFFBOARD while PX4 maintains stabilization. No trajectory time
        // elapses yet.
        out.type_mask = Target::IGNORE_PX | Target::IGNORE_PY | Target::IGNORE_PZ |
                        Target::IGNORE_AFX | Target::IGNORE_AFY | Target::IGNORE_AFZ |
                        Target::IGNORE_YAW_RATE;
      }
      if (!active && t - stream_started_ > 5) {
        navigation_fault_ = true;
        operation("HOLD");
        stream_started_ = 0;
        return;
      }
      if (active && offboard_start_ == 0)
        offboard_start_ = t;
      if (active && !tracking &&
          ((had_control_ && t - command_time_ > .15) || command_.braking ||
           t - offboard_start_ > 2)) {
        navigation_fault_ = true;
        operation("HOLD");
        stream_started_ = 0;
        had_control_ = false;
      } else {
        out_->publish(out);
        if (!active && t - stream_started_ > 1.2 && t - last_mode_request_ > 2) {
          set_mode("OFFBOARD");
          last_mode_request_ = t;
        }
      }
    } else {
      stream_started_ = offboard_start_ = 0;
      had_control_ = false;
      if (state_.mode == "OFFBOARD" && !land_latched_ && !externally_armed_ &&
          t - last_mode_request_ > 1) {
        set_mode("AUTO.LOITER");
        last_mode_request_ = t;
      }
    }
    if (t - last_status_ > .1) {
      last_status_ = t;
      auto caps = capabilities();
      auto f = feedback();
      robotics::flight::Json status = {
          {"connected", f.connected},
          {"ready", ready()},
          {"armed", f.armed},
          {"ground", f.on_ground},
          {"airborne", f.airborne},
          {"emergency", emergency_},
          {"takeoff_allowed", ready() && authorized_ && t - authorization_time_ < .3 && ground() &&
                                  state_.armed && arm_requested_ && !takeoff_requested_ &&
                                  std::find(takeoff_modes_.begin(), takeoff_modes_.end(),
                                            state_.mode) != takeoff_modes_.end()},
          {"landing", land_latched_},
          {"identity_verified", identity_},
          {"parameters_verified", parameters_},
          {"pilot_enabled", pilot_ && t - rc_time_ < .3},
          {"localization_valid", localized_ && t - estimate_time_ < .25},
          {"sensors_available", t - map_time_ < .3},
          {"mode", state_.mode},
          {"phase", f.phase},
          {"navigation_active", active},
          {"navigation_fault", navigation_fault_},
          {"blocked_reasons", blocked()},
          {"detail", detail_},
          {"capabilities",
           {{"backend", "px4_mavros"},
            {"protocol_verified", caps.protocol_verified},
            {"takeoff", caps.takeoff},
            {"acceleration", caps.acceleration},
            {"hold", caps.hold},
            {"land", caps.land},
            {"ground_feedback", caps.ground_feedback}}}};
      std_msgs::msg::String m;
      m.data = status.dump();
      status_->publish(m);
    }
  }
  std::string expected_mode_;
  std::map<std::string, double> expected_exits_;
  std::string uid_,
      detail_{"BLOCKED: awaiting live identity, parameters, RC, stereo and localization"};
  int64_t version_;
  int rc_channel_, rc_min_, rc_max_;
  double max_acc_, vo_time_{}, vo_stamp_{}, orientation_time_{}, yaw_{}, hold_yaw_{},
      offboard_start_{}, authorization_time_{}, takeoff_start_{};
  bool authorized_{}, navigation_fault_{}, had_control_{};
  geometry_msgs::msg::Quaternion vo_orientation_;
  std::vector<std::string> takeoff_modes_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr authorization_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_;
  std::vector<std::string> param_names_;
  std::vector<double> param_values_;
  bool param_pending_{};
  bool identity_{}, parameters_{}, pilot_{}, localized_{}, permit_{}, emergency_{}, land_latched_{},
      arm_requested_{}, takeoff_requested_{}, last_armed_{}, externally_armed_{}, offboard_seen_{},
      mode_exit_authorized_{};
  geometry_msgs::msg::PoseStamped local_;
  double local_time_{}, imu_time_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<double> covariance_;
  bool vo_valid_{}, orientation_agrees_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double identity_time_{}, parameters_time_{};
  double state_time_{}, ext_time_{}, rc_time_{}, estimate_time_{}, map_time_{}, permit_time_{},
      command_time_{}, command_stamp_{}, stream_started_{}, last_mode_request_{}, last_status_{};
  mavros_msgs::msg::State state_;
  mavros_msgs::msg::ExtendedState ext_;
  flight_interfaces::msg::ControlCommand command_;
  rclcpp::Publisher<Target>::SharedPtr out_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Client<Mode>::SharedPtr mode_;
  rclcpp::Client<Arm>::SharedPtr arm_;
  rclcpp::Client<mavros_msgs::srv::VehicleInfoGet>::SharedPtr info_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<flight_interfaces::msg::VehicleState>::SharedPtr estimate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permit_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr op_sub_;
  rclcpp::Subscription<flight_interfaces::msg::ControlCommand>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr timer_, verify_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightAdapter>());
  rclcpp::shutdown();
}
