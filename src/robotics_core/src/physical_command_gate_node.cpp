#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "flight_interfaces/msg/control_command.hpp"
#include "flight_interfaces/msg/flight_controller_status.hpp"
#include "flight_interfaces/msg/msp_raw_telemetry.hpp"
#include "flight_interfaces/msg/timed_trajectory.hpp"
#include "flight_interfaces/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotics_core/command_gate.hpp"

class PhysicalCommandGateNode final : public rclcpp::Node {
 public:
  PhysicalCommandGateNode() : Node("physical_command_gate") {
    declare_parameter("operator_authorized", false);
    declare_parameter("emergency_stop", true);
    robotics::control::GateLimits limits;
    limits.command_age_s = declare_parameter("command_age_s", .15);
    limits.acceleration_max_mps2 = declare_parameter("acceleration_max_mps2", 2.0);
    limits.telemetry_age_s = declare_parameter("telemetry_age_s", .25);
    limits.estimate_age_s = declare_parameter("estimate_age_s", .25);
    gate_ = robotics::control::CommandGate(limits);
    if (declare_parameter("hardware_output_enabled", false))
      throw std::invalid_argument("physical command transmission is unavailable in Phase 5");
    auto sensor = rclcpp::SensorDataQoS();
    command_ = create_subscription<flight_interfaces::msg::ControlCommand>(
        "control/acceleration", rclcpp::QoS(1).best_effort(),
        [this](flight_interfaces::msg::ControlCommand::SharedPtr m) {
          last_command_ = robotics::control::CommandSample{
              rclcpp::Time(m->header.stamp).seconds(), m->trajectory_id,
              {m->acceleration.x, m->acceleration.y, m->acceleration.z},
              m->braking, m->header.frame_id};
          report();
        });
    state_ = create_subscription<flight_interfaces::msg::VehicleState>(
        "perception/state_estimate", sensor,
        [this](flight_interfaces::msg::VehicleState::SharedPtr m) {
          health_.estimate_healthy = m->valid && !m->ground_truth && m->header.frame_id == "map";
          health_.estimate_stamp = rclcpp::Time(m->header.stamp).seconds();
        });
    trajectory_ = create_subscription<flight_interfaces::msg::TimedTrajectory>(
        "planning/trajectory", rclcpp::QoS(1).transient_local().reliable(),
        [this](flight_interfaces::msg::TimedTrajectory::SharedPtr m) {
          health_.trajectory_valid = m->trajectory_id != 0 && !m->points.empty() &&
                                     std::isfinite(m->duration) && m->duration > 0 &&
                                     m->header.frame_id == "map";
          health_.trajectory_start = rclcpp::Time(m->header.stamp).seconds();
          health_.trajectory_end = health_.trajectory_start + m->duration;
          health_.trajectory_id = m->trajectory_id;
        });
    controller_ = create_subscription<flight_interfaces::msg::FlightControllerStatus>(
        "flight_controller/status", 10,
        [this](flight_interfaces::msg::FlightControllerStatus::SharedPtr m) {
          health_.controller_connected = m->connected;
          health_.telemetry_stamp = rclcpp::Time(m->stamp).seconds();
        });
    telemetry_ = create_subscription<flight_interfaces::msg::MspRawTelemetry>(
        "flight_controller/raw_telemetry", 10,
        [this](flight_interfaces::msg::MspRawTelemetry::SharedPtr m) {
          health_.telemetry_valid = m->ident_available && m->status_available;
          health_.telemetry_stamp = rclcpp::Time(m->stamp).seconds();
        });
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { report(); });
  }
 private:
  void report() {
    health_.operator_authorized = get_parameter("operator_authorized").as_bool();
    health_.emergency_stop = get_parameter("emergency_stop").as_bool();
    // No verified firmware mapping, mode, or failsafe is available in this phase.
    health_.protocol_verified = false;
    health_.configuration_verified = false;
    health_.operating_mode_known = false;
    std::string reason = last_command_ ? gate_.check_physical(*last_command_, now().seconds(), health_)
                                       : "no command received";
    if (reason.empty()) reason = "physical encoding and transmission unavailable";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "physical_command_gate";
    status.hardware_id = "unverified-flight-controller";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "CLOSED: " + reason;
    diagnostic_msgs::msg::KeyValue item;
    item.key = "actuator_transmission";
    item.value = "disabled";
    status.values.push_back(item);
    array.status.push_back(status);
    diagnostics_->publish(array);
  }
  robotics::control::CommandGate gate_;
  robotics::control::PhysicalHealth health_;
  std::optional<robotics::control::CommandSample> last_command_;
  rclcpp::Subscription<flight_interfaces::msg::ControlCommand>::SharedPtr command_;
  rclcpp::Subscription<flight_interfaces::msg::VehicleState>::SharedPtr state_;
  rclcpp::Subscription<flight_interfaces::msg::TimedTrajectory>::SharedPtr trajectory_;
  rclcpp::Subscription<flight_interfaces::msg::FlightControllerStatus>::SharedPtr controller_;
  rclcpp::Subscription<flight_interfaces::msg::MspRawTelemetry>::SharedPtr telemetry_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PhysicalCommandGateNode>());
  rclcpp::shutdown();
  return 0;
}
