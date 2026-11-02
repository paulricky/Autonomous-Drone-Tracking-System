// Process lifecycle only. Flight commands and watchdogs live in separate native nodes.
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robotics_core/operator_session.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
static volatile std::sig_atomic_t stop_requested = 0;
static void request_stop(int) {
  stop_requested = 1;
}
class DroneSupervisor final : public rclcpp::Node {
 public:
  DroneSupervisor() : Node("drone_application_supervisor") {
    auto config = declare_parameter("config_file", std::string{});
    if (config.empty())
      throw std::runtime_error("physical configuration path required");
    emergency_ = create_publisher<std_msgs::msg::Empty>("flight/emergency_request",
                                                        rclcpp::QoS(1).reliable());
    feedback_ = create_subscription<std_msgs::msg::String>(
        "flight/status", 10, [this](std_msgs::msg::String::SharedPtr m) {
          try {
            status_ = robotics::flight::Json::parse(m->data);
            receipt_ = std::chrono::steady_clock::now();
            auto text = status_.dump();
            if (text != last_status_) {
              RCLCPP_INFO(get_logger(), "%s", text.c_str());
              last_status_ = text;
            }
          } catch (const std::exception&) {
          }
        });
    std::string arg = "config:=" + config;
    child_ = fork();
    if (child_ < 0)
      throw std::runtime_error("cannot fork physical launch");
    if (child_ == 0) {
      setsid();
      signal(SIGINT, SIG_DFL);
      signal(SIGTERM, SIG_DFL);
      execlp("ros2", "ros2", "launch", "robotics_bringup", "physical_flight.launch.py", arg.c_str(),
             nullptr);
      _exit(127);
    }
    timer_ = create_wall_timer(50ms, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "Application started; hardware readiness pending. No automatic arming or takeoff.");
  }
  int exit_code() const {
    return exit_code_;
  }

 private:
  void tick() {
    int status = 0;
    auto result = waitpid(child_, &status, WNOHANG);
    if (result == child_) {
      exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
      rclcpp::shutdown();
      return;
    }
    if (!stop_requested)
      return;
    bool safe = false;
    try {
      safe =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - receipt_).count() < .4 &&
          status_.value("connected", false) && status_.value("identity_verified", false) &&
          status_.value("ground", false) && status_.contains("armed") && status_["armed"] == false;
    } catch (const std::exception&) {
    }
    if (safe) {
      if (!exit_sent_) {
        kill(-child_, SIGINT);
        exit_sent_ = true;
      }
    } else {
      emergency_->publish(std_msgs::msg::Empty{});
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Shutdown pending verified ground/disarming feedback; emergency "
                           "recovery requested; independent pilot recovery remains available");
    }
  }
  pid_t child_{};
  int exit_code_{1};
  bool exit_sent_{};
  robotics::flight::Json status_ = robotics::flight::Json::object();
  std::string last_status_;
  std::chrono::steady_clock::time_point receipt_{};
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr emergency_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto supervisor = std::make_shared<DroneSupervisor>();
  rclcpp::spin(supervisor);
  return supervisor->exit_code();
}
