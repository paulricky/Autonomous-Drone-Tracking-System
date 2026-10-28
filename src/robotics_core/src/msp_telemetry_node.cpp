#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "flight_interfaces/msg/flight_controller_status.hpp"
#include "flight_interfaces/msg/msp_raw_telemetry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotics_core/msp_serial.hpp"

using namespace std::chrono_literals;
class MspTelemetryNode final : public rclcpp::Node {
 public:
  MspTelemetryNode() : Node("msp_telemetry") {
    enabled_ = declare_parameter("transport_enabled", false);
    const auto device = declare_parameter("serial_device", "");
    const auto baud = declare_parameter("baud_rate", 115200);
    const auto protocol = declare_parameter("protocol", "unverified");
    const auto timestamp_policy = declare_parameter("timestamp_policy", "ros_poll_completion");
    if (timestamp_policy != "ros_poll_completion")
      throw std::invalid_argument("unsupported MSP timestamp policy");
    if (declare_parameter("hardware_output_enabled", false))
      throw std::invalid_argument("MSP telemetry node has no actuator output path");
    if (enabled_) {
      if (protocol != "legacy_msp_v1_candidate" || device.empty() ||
          device.find("UNCONFIGURED") != std::string::npos ||
          !std::filesystem::exists(device) ||
          (device.rfind("/dev/serial/by-id/", 0) != 0 &&
           device.rfind("/dev/serial/by-path/", 0) != 0))
        throw std::invalid_argument("read-only MSP requires an explicit candidate protocol and existing serial device");
      serial_ = std::make_unique<robotics::msp::SerialTelemetry>(device, baud);
    }
    status_ = create_publisher<flight_interfaces::msg::FlightControllerStatus>(
        "flight_controller/status", 10);
    raw_ = create_publisher<flight_interfaces::msg::MspRawTelemetry>(
        "flight_controller/raw_telemetry", 10);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(1s, [this] { tick(); });
  }
 private:
  void tick() {
    flight_interfaces::msg::MspRawTelemetry raw;
    std::string detail = "transport disabled; no serial device opened";
    if (serial_) {
      if (!serial_->connected() && !serial_->open_port()) {
        detail = "serial unavailable; reconnect pending";
      } else {
        const auto ident_packet = serial_->query(robotics::msp::kIdent, 100ms);
        const auto ident = ident_packet ? robotics::msp::decode_ident(*ident_packet) : std::nullopt;
        if (ident) {
          raw.ident_available = true;
          raw.firmware_version = ident->firmware_version;
          raw.vehicle_type = ident->vehicle_type;
          raw.msp_version = ident->msp_version;
          raw.capability = ident->capability;
          ++valid_packets_;
          detail = "candidate MSP v1 IDENT received; firmware identity and physical control remain unverified";
          const auto status_packet = serial_->query(robotics::msp::kStatus, 100ms);
          const auto status = status_packet ? robotics::msp::decode_status(*status_packet) : std::nullopt;
          if (status) {
            raw.status_available = true;
            raw.cycle_time_us = status->cycle_time_us;
            raw.i2c_errors = status->i2c_errors;
            raw.sensor_flags = status->sensor_flags;
            raw.mode_flags = status->mode_flags;
            raw.profile = status->profile;
            ++valid_packets_;
          }
          const auto analog_packet = serial_->query(robotics::msp::kAnalog, 100ms);
          const auto analog = analog_packet ? robotics::msp::decode_analog(*analog_packet) : std::nullopt;
          if (analog) {
            raw.analog_available = true;
            raw.battery_raw = analog->battery_raw;
            raw.power_meter_raw = analog->power_meter;
            raw.rssi_raw = analog->rssi;
            raw.current_raw = analog->current_raw;
            ++valid_packets_;
          }
          const auto imu = serial_->query(robotics::msp::kRawImu, 100ms);
          if (imu && !imu->error && imu->payload.size() == 18) {
            raw.raw_imu_available = true;
            for (std::size_t i = 0; i < 9; ++i) {
              const int bits = int(imu->payload[2 * i]) |
                               (int(imu->payload[2 * i + 1]) << 8);
              raw.raw_imu[i] = static_cast<std::int16_t>(bits < 32768 ? bits : bits - 65536);
            }
            ++valid_packets_;
          }
        } else {
          detail = ident_packet && ident_packet->error
                       ? "controller explicitly rejected MSP v1 IDENT; check firmware and port protocol"
                       : "MSP IDENT unavailable or malformed; check baud, wiring and firmware protocol";
          if (++missed_ident_ >= 3) {
            serial_->close_port();
            missed_ident_ = 0;
          }
        }
        if (ident) missed_ident_ = 0;
      }
    }
    // This aggregate is stamped at host poll completion, never as controller
    // sensor acquisition time. Individual replies may span several queries.
    raw.stamp = now();
    raw_->publish(raw);
    flight_interfaces::msg::FlightControllerStatus status;
    status.stamp = raw.stamp;
    status.connected = raw.ident_available && serial_ && serial_->connected();
    status.hardware_output_enabled = false;
    status.command_gate_open = false;
    status.valid_packets = valid_packets_;
    status.invalid_packets = serial_ ? serial_->invalid_packets() : 0;
    status.command_rate_hz = 0;
    status.detail = detail;
    status_->publish(status);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = raw.stamp;
    diagnostic_msgs::msg::DiagnosticStatus d;
    d.name = "msp_read_only_telemetry";
    d.hardware_id = "unverified-flight-controller";
    d.level = status.connected ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                               : diagnostic_msgs::msg::DiagnosticStatus::STALE;
    d.message = detail;
    diagnostics.status.push_back(d);
    diagnostics_->publish(diagnostics);
  }
  bool enabled_{};
  std::uint32_t valid_packets_{}, missed_ident_{};
  std::unique_ptr<robotics::msp::SerialTelemetry> serial_;
  rclcpp::Publisher<flight_interfaces::msg::FlightControllerStatus>::SharedPtr status_;
  rclcpp::Publisher<flight_interfaces::msg::MspRawTelemetry>::SharedPtr raw_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<MspTelemetryNode>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("msp_telemetry"), "%s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
