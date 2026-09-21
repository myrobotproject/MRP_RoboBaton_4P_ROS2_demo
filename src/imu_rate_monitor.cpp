#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#ifndef ROBOBATON_RELEASE_VERSION
#define ROBOBATON_RELEASE_VERSION "0.0.0+unknown"
#endif

namespace {

constexpr char kDefaultImuTopic[] = "/robobaton/imu/data";

uint32_t DeclarePositiveUint32(rclcpp::Node* node, const std::string& name,
                               uint32_t default_value) {
  const int64_t value =
      node->declare_parameter<int64_t>(name, static_cast<int64_t>(default_value));
  if (value <= 0 || value > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
    throw std::invalid_argument(name + " must be a positive uint32");
  }
  return static_cast<uint32_t>(value);
}

std::string DeclareTopic(rclcpp::Node* node) {
  const std::string topic = node->declare_parameter<std::string>("topic", kDefaultImuTopic);
  if (topic.empty() || topic.front() != '/' ||
      topic.find_first_of(" \t\r\n") != std::string::npos) {
    throw std::invalid_argument("topic must be an absolute ROS topic name without whitespace");
  }
  return topic;
}

class ImuRateMonitor final : public rclcpp::Node {
 public:
  ImuRateMonitor() : rclcpp::Node("robobaton_imu_rate_monitor") {
    topic_ = DeclareTopic(this);
    report_period_ms_ = DeclarePositiveUint32(this, "report_period_ms", 1000U);
    qos_depth_ = DeclarePositiveUint32(this, "qos_depth", 100U);

    // The callback only increments an atomic counter; the timer computes the rate and writes output so each IMU sample avoids formatting overhead.
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        topic_, rclcpp::SensorDataQoS().keep_last(qos_depth_),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr) {
          total_count_.fetch_add(1U, std::memory_order_relaxed);
        });

    last_report_time_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(report_period_ms_),
                               [this] { Report(); });
    RCLCPP_INFO(get_logger(), "Monitoring IMU topic %s with C++ subscriber", topic_.c_str());
  }

 private:
  void Report() {
    const auto now = std::chrono::steady_clock::now();
    const double window_s = std::chrono::duration<double>(now - last_report_time_).count();
    const uint64_t current_total = total_count_.load(std::memory_order_relaxed);
    const uint64_t samples = current_total - last_total_;
    const double hz = window_s > 0.0 ? static_cast<double>(samples) / window_s : 0.0;
    last_report_time_ = now;
    last_total_ = current_total;

    RCLCPP_INFO(get_logger(),
                "ROB2_IMU_RATE topic=%s hz=%.3f samples=%llu window_s=%.6f total=%llu",
                topic_.c_str(), hz, static_cast<unsigned long long>(samples), window_s,
                static_cast<unsigned long long>(current_total));
  }

  std::string topic_;
  uint32_t report_period_ms_ = 1000U;
  uint32_t qos_depth_ = 100U;
  std::atomic<uint64_t> total_count_{0U};
  uint64_t last_total_ = 0U;
  std::chrono::steady_clock::time_point last_report_time_{};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--version") {
    std::printf("robobaton_imu_rate_monitor %s\n", ROBOBATON_RELEASE_VERSION);
    return 0;
  }
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<ImuRateMonitor>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "fatal: %s\n", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
