#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;
class HardwareMonitor final : public rclcpp_lifecycle::LifecycleNode {
 public:
  HardwareMonitor() : LifecycleNode("hardware_monitor") {
    declare_parameter("hardware_output_enabled", false);
    declare_parameter("serial_device", "");
    declare_parameter("baud_rate", 115200);
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    enabled_ = get_parameter("hardware_output_enabled").as_bool();
    if (enabled_) {
      RCLCPP_ERROR(
          get_logger(),
          "Physical output requires a separately reviewed transport; refusing configuration");
      return CallbackReturn::FAILURE;
    }
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(1s, [this] { publish_status(); });
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    timer_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    publisher_->on_activate();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

 private:
  void publish_status() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "ece3162/hardware_output";
    status.hardware_id = "sp-racing-f3-unconnected";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "hardware disabled by default; no serial device opened";
    diagnostic_msgs::msg::KeyValue gate;
    gate.key = "command_gate";
    gate.value = enabled_ ? "closed-invalid-configuration" : "closed";
    status.values.push_back(gate);
    array.status.push_back(status);
    publisher_->publish(array);
  }
  bool enabled_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareMonitor>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
