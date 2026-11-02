// PX4 stabilized acceleration interface through MAVROS. No raw RC/motor output.
#include <chrono>
#include <cmath>
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
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
class FlightAdapter : public rclcpp::Node {
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
    max_acc_ = declare_parameter("max_acceleration_mps2", 0.0);
    param_names_ = declare_parameter("verified_parameter_names", std::vector<std::string>{});
    param_values_ = declare_parameter("verified_parameter_values", std::vector<double>{});
    if (uid_ == "UNCONFIGURED" || version_ <= 0 || rc_channel_ < 0 || rc_min_ <= 0 ||
        rc_max_ <= rc_min_ || max_acc_ <= 0 || param_names_.size() != param_values_.size() ||
        param_names_.empty())
      throw std::runtime_error(
          "commissioned PX4 identity, pilot channel, limits and failsafe parameters required");
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
          if (!vo_valid_ || age < 0 || age > .25 || m->header.frame_id != "map")
            return;
          const auto q = m->pose.pose.orientation;
          const auto l = local_.pose.orientation;
          double dot = std::abs(q.x * l.x + q.y * l.y + q.z * l.z + q.w * l.w);
          orientation_agrees_ = std::isfinite(dot) && dot > std::cos(.1) && dot < 1.001;
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
          if (offboard_seen_ && state_.mode == "OFFBOARD" && m->mode != "OFFBOARD" &&
              !mode_exit_authorized_)
            externally_armed_ =
                true;  // Pilot/controller mode change latches autonomous output off.
          state_ = *m;
          state_time_ = steady();
          if (state_.mode == "OFFBOARD") {
            offboard_seen_ = true;
            mode_exit_authorized_ = false;
          }
          if (state_.armed && !last_armed_)
            externally_armed_ = !arm_requested_;
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
          pilot_ = size_t(rc_channel_) < m->channels.size() &&
                   m->channels[rc_channel_] >= rc_min_ && m->channels[rc_channel_] <= rc_max_;
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "mavros/imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::SharedPtr m) {
          double age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
          if (age >= 0 && age < .25 && m->orientation_covariance[0] >= 0)
            imu_time_ = steady();
        });
    local_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          local_ = *m;
          local_time_ = steady();
        });
    estimate_sub_ = create_subscription<flight_interfaces::msg::VehicleState>(
        "perception/state_estimate", rclcpp::SensorDataQoS(),
        [this](flight_interfaces::msg::VehicleState::SharedPtr m) {
          auto age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
          localized_ =
              m->valid && !m->ground_truth && m->header.frame_id == "map" && age >= 0 && age < .25;
          vo_valid_ = localized_;
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
          double age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
          auto a = m->acceleration;
          if (age < 0 || age > .15 || m->header.frame_id != "map" || !std::isfinite(a.x) ||
              !std::isfinite(a.y) || !std::isfinite(a.z) ||
              std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z) > max_acc_ ||
              rclcpp::Time(m->header.stamp).seconds() <= command_stamp_)
            return;
          command_stamp_ = rclcpp::Time(m->header.stamp).seconds();
          command_ = *m;
          command_time_ = steady();
        });
    timer_ = create_wall_timer(20ms, [this] { tick(); });
    verify_ = create_wall_timer(2s, [this] { verify(); });
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
           steady() - imu_time_ < .25 && orientation_agrees_ && !emergency_ && !land_latched_ &&
           !externally_armed_;
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
    add(!externally_armed_, "pilot/controller takeover latched");
    return reasons;
  }
  bool ground() const {
    return connected() &&
           ext_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
  }
  void set_mode(const std::string& name) {
    if (name != "OFFBOARD")
      mode_exit_authorized_ = true;
    if (!mode_->service_is_ready()) {
      detail_ = "REJECTED mode service unavailable";
      return;
    }
    auto r = std::make_shared<Mode::Request>();
    r->custom_mode = name;
    mode_->async_send_request(r, [this, name](rclcpp::Client<Mode>::SharedFuture f) {
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
      if (connected() && identity_ && parameters_ && !externally_armed_ && state_.armed)
        set_mode("AUTO.LAND");
      else
        detail_ = "REJECTED LAND: verified controller unavailable";
      return;
    }
    if (op == "HOLD") {
      permit_ = false;
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
    if (op == "ARM" && ready() && ground() && !state_.armed && !arm_requested_ && !land_latched_) {
      arm_requested_ = true;
      arm(true);
      return;
    }
    if (op == "TAKEOFF" && ready() && ground() && state_.armed && arm_requested_ &&
        !takeoff_requested_ && !land_latched_) {
      takeoff_requested_ = true;
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
    r->get_all = true;
    info_->async_send_request(
        r, [this](rclcpp::Client<mavros_msgs::srv::VehicleInfoGet>::SharedFuture f) {
          identity_time_ = steady();
          identity_ = false;
          for (auto& v : f.get()->vehicles)
            if (v.autopilot == 12 && v.type == 2 && std::to_string(v.uid) == uid_ &&
                v.flight_sw_version == version_ && (v.available_info & 2))
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
    if (state_.armed && takeoff_requested_ && !externally_armed_ &&
        (!connected() || !identity_ || !parameters_ || !pilot_ || t - rc_time_ > .3 ||
         !localized_ || t - estimate_time_ > .25 || t - map_time_ > .3 || t - permit_time_ > .5)) {
      if (!land_latched_)
        operation("EMERGENCY");
    }
    bool nav = ready() && state_.armed && takeoff_requested_ && permitted && !land_latched_ &&
               ext_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
    if (nav && t - command_time_ < .15 && !command_.braking) {
      Target out;
      out.header.stamp = now();
      out.header.frame_id = "map";
      out.coordinate_frame = Target::FRAME_LOCAL_NED;
      out.type_mask = Target::IGNORE_PX | Target::IGNORE_PY | Target::IGNORE_PZ |
                      Target::IGNORE_VX | Target::IGNORE_VY | Target::IGNORE_VZ |
                      Target::IGNORE_YAW | Target::IGNORE_YAW_RATE;
      // Input is net translational ENU m/s², FORCE bit clear. MAVROS converts ENU to NED;
      // PX4 performs gravity compensation, thrust/attitude allocation and stabilization.
      out.acceleration_or_force = command_.acceleration;
      out_->publish(out);
      if (stream_started_ == 0)
        stream_started_ = t;
      if (state_.mode != "OFFBOARD" && t - stream_started_ > 1.2 && t - last_mode_request_ > 2) {
        set_mode("OFFBOARD");
        last_mode_request_ = t;
      }
    } else {
      stream_started_ = 0;
      if (state_.mode == "OFFBOARD" && !land_latched_ && t - last_mode_request_ > 1) {
        set_mode("AUTO.LOITER");
        last_mode_request_ = t;
      }
      // Stop offboard heartbeats: PX4's commissioned offboard-loss failsafe remains independent.
    }
    if (t - last_status_ > .1) {
      last_status_ = t;
      std::ostringstream s;
      s << "{\"connected\":" << (connected() ? "true" : "false")
        << ",\"ready\":" << (ready() ? "true" : "false")
        << ",\"armed\":" << (state_.armed ? "true" : "false")
        << ",\"ground\":" << (ground() ? "true" : "false")
        << ",\"emergency\":" << (emergency_ ? "true" : "false")
        << ",\"landing\":" << (land_latched_ ? "true" : "false")
        << ",\"airborne\":" << ((connected() && ext_.landed_state == 2) ? "true" : "false")
        << ",\"identity_verified\":" << (identity_ ? "true" : "false")
        << ",\"parameters_verified\":" << (parameters_ ? "true" : "false")
        << ",\"pilot_enabled\":" << (pilot_ && t - rc_time_ < .3 ? "true" : "false")
        << ",\"localization_valid\":" << (localized_ && t - estimate_time_ < .25 ? "true" : "false")
        << ",\"sensors_available\":" << (t - map_time_ < .3 ? "true" : "false") << ",\"mode\":\""
        << state_.mode << "\",\"blocked_reasons\":\"" << blocked() << "\",\"detail\":\"" << detail_
        << "\"}";
      std_msgs::msg::String m;
      m.data = s.str();
      status_->publish(m);
    }
  }
  std::string uid_,
      detail_{"BLOCKED: awaiting live identity, parameters, RC, stereo and localization"};
  int64_t version_;
  int rc_channel_, rc_min_, rc_max_;
  double max_acc_;
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
