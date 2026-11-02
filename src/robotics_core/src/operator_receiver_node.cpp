#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotics_core/operator_session.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
namespace flight = robotics::flight;
class OperatorReceiver final : public rclcpp::Node {
 public:
  OperatorReceiver()
      : Node("operator_receiver"),
        session_(declare_parameter("reference_frame_id", std::string{})) {
    auto path = declare_parameter("key_file", std::string{});
    struct stat info {};
    if (stat(path.c_str(), &info) != 0 || !S_ISREG(info.st_mode) || (info.st_mode & 0077))
      throw std::runtime_error("operator key must be an owner-only regular file: " + path);
    std::ifstream file(path, std::ios::binary);
    key_ = {std::istreambuf_iterator<char>(file), {}};
    auto first = key_.find_first_not_of(" \t\r\n\v\f"), last = key_.find_last_not_of(" \t\r\n\v\f");
    key_ = first == std::string::npos ? "" : key_.substr(first, last - first + 1);
    if (key_.size() < 32)
      throw std::runtime_error("operator key requires at least 32 random bytes");
    auto address = declare_parameter("bind_address", std::string("0.0.0.0"));
    auto port = declare_parameter("port", 7447);
    if (port < 1 || port > 65535)
      throw std::invalid_argument("invalid operator UDP port");
    sockaddr_in bind_address{};
    bind_address.sin_family = AF_INET;
    bind_address.sin_port = htons(port);
    if (inet_pton(AF_INET, address.c_str(), &bind_address.sin_addr) != 1)
      throw std::invalid_argument("invalid UDP bind address");
    fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0 || bind(fd_, reinterpret_cast<sockaddr*>(&bind_address), sizeof(bind_address)) != 0)
      throw std::runtime_error(
          "operator UDP bind failed; check address/port and existing applications");
    operation_ = create_publisher<std_msgs::msg::String>("flight/operator_request",
                                                         rclcpp::QoS(1).reliable());
    emergency_ = create_publisher<std_msgs::msg::Empty>("flight/emergency_request",
                                                        rclcpp::QoS(1).reliable());
    authorization_ = create_publisher<std_msgs::msg::Bool>("mission/operator_authorized", 1);
    permit_ = create_publisher<std_msgs::msg::Bool>("mission/navigation_enabled", 1);
    stop_ = create_publisher<std_msgs::msg::Empty>("mission/navigation_stop", 1);
    direction_ = create_publisher<geometry_msgs::msg::Vector3>("mission/direction", 1);
    feedback_ = create_subscription<std_msgs::msg::String>(
        "flight/status", 10, [this](std_msgs::msg::String::SharedPtr m) {
          try {
            session_.observe(flight::Json::parse(m->data), steady());
          } catch (const flight::Json::exception&) {
          }
        });
    navigation_ = create_subscription<std_msgs::msg::String>(
        "mission/status", 10,
        [this](std_msgs::msg::String::SharedPtr m) { navigation_status_ = m->data; });
    timer_ = create_wall_timer(5ms, [this] { poll(); });
    RCLCPP_INFO(get_logger(),
                "Native authenticated operator gateway listening on %s:%d; motors unchanged",
                address.c_str(), int(port));
  }
  ~OperatorReceiver() override {
    if (fd_ >= 0)
      close(fd_);
  }

 private:
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  static double wall() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
        .count();
  }
  bool same_peer(const sockaddr_in& peer) const {
    return active_ && peer.sin_port == peer_.sin_port &&
           peer.sin_addr.s_addr == peer_.sin_addr.s_addr;
  }
  void reply(const std::string& result) {
    if (!active_)
      return;
    auto response = session_.response(steady());
    response.update({{"time", wall()},
                     {"session", token_},
                     {"seq", ++reply_sequence_},
                     {"request_seq", request_sequence_},
                     {"result", result},
                     {"navigation", navigation_status_}});
    auto packet = flight::encode_operator_packet(key_, response);
    sendto(fd_, packet.data(), packet.size(), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&peer_),
           sizeof(peer_));
  }
  void execute(const flight::OperatorDecision& decision) {
    if (!decision.operation.empty()) {
      // Revoke navigation before transmitting any recovery operation.
      std_msgs::msg::Bool permit;
      permit.data = false;
      permit_->publish(permit);
      if (session_.latched())
        stop_->publish(std_msgs::msg::Empty{});
      if (decision.operation == "EMERGENCY")
        emergency_->publish(std_msgs::msg::Empty{});
      else {
        std_msgs::msg::String operation;
        operation.data = decision.operation;
        operation_->publish(operation);
      }
    }
    if (decision.direction) {
      std_msgs::msg::Bool permit;
      permit.data = true;
      permit_->publish(permit);
      geometry_msgs::msg::Vector3 d;
      d.x = (*decision.direction)[0];
      d.y = (*decision.direction)[1];
      d.z = (*decision.direction)[2];
      direction_->publish(d);
    }
  }
  void poll() {
    std::vector<flight::Json> packets;
    for (int i = 0; i < 64; ++i) {
      char buffer[8193];
      sockaddr_in peer{};
      socklen_t size = sizeof(peer);
      auto count = recvfrom(fd_, buffer, sizeof(buffer), MSG_DONTWAIT,
                            reinterpret_cast<sockaddr*>(&peer), &size);
      if (count < 0)
        break;
      try {
        auto m = flight::decode_operator_packet(key_, std::string(buffer, count), wall());
        if (!m.contains("command") || !m["command"].is_string())
          continue;
        if (m["command"] == "HELLO") {
          if (same_peer(peer) && steady() - last_packet_ < .7)
            reply("connected");
          else if (steady() - last_packet_ > .7) {
            // Apply loss policy before giving any new connection a lease.
            execute(session_.tick(steady()));
            peer_ = peer;
            active_ = true;
            token_ = flight::random_token();
            sequence_ = -1;
            last_packet_ = steady();
            session_.connected(last_packet_);
            reply("connected; press S to enable");
          }
          continue;
        }
        if (!same_peer(peer) || !m.contains("session") || m["session"] != token_ ||
            !m.contains("seq") || !m["seq"].is_number_integer())
          continue;
        auto seq = m["seq"].get<int64_t>();
        if (seq <= sequence_ || seq < 0)
          continue;
        packets.push_back(std::move(m));
      } catch (const std::exception&) {
        continue;
      }
    }
    std::sort(packets.begin(), packets.end(), [](const auto& a, const auto& b) {
      return a.at("seq").template get<int64_t>() < b.at("seq").template get<int64_t>();
    });
    auto emergency = std::find_if(packets.rbegin(), packets.rend(),
                                  [](const auto& m) { return m.at("command") == "EMERGENCY"; });
    if (emergency != packets.rend()) {
      auto m = *emergency;
      auto high = packets.back().at("seq").get<int64_t>();
      packets = {m};
      sequence_ = high;
      request_sequence_ = m["seq"].get<int64_t>();
      last_packet_ = steady();
      execute(session_.handle(m, last_packet_));
      reply("EMERGENCY latched; controller feedback pending");
      packets.clear();
    }
    for (const auto& m : packets) {
      auto seq = m["seq"].get<int64_t>();
      if (seq <= sequence_)
        continue;
      sequence_ = request_sequence_ = seq;
      last_packet_ = steady();
      try {
        auto decision = session_.handle(m, last_packet_);
        execute(decision);
        reply(decision.result);
      } catch (const std::exception&) {
        reply("REJECTED malformed operator command");
      }
    }
    auto decision = session_.tick(steady());
    execute(decision);
    if (!decision.result.empty())
      reply(decision.result);
    std_msgs::msg::Bool authorized;
    authorized.data = session_.authorized();
    authorization_->publish(authorized);
    std_msgs::msg::Bool permit;
    permit.data = session_.navigation_permitted(steady());
    permit_->publish(permit);
  }
  flight::OperatorSession session_;
  int fd_{-1};
  bool active_{};
  sockaddr_in peer_{};
  std::string key_, token_, navigation_status_{"waiting"};
  int64_t sequence_{-1}, request_sequence_{-1}, reply_sequence_{};
  double last_packet_{-1e9};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr emergency_, stop_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permit_, authorization_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr direction_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_, navigation_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OperatorReceiver>());
  rclcpp::shutdown();
}
