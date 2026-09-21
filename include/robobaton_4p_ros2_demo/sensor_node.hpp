#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robobaton_4p_ros2_demo/camera_publisher.hpp"
#include "robobaton_4p_ros2_demo/imu_publisher.hpp"
#include "robobaton_4p_ros2_demo/timestamp_mapper.hpp"

namespace robobaton_4p_ros2_demo {

class SensorNode : public rclcpp::Node {
 public:
  explicit SensorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SensorNode() override;

 private:
  void StartEnabledPublishers();
  void StopPublishers();
  CameraPublisher::Config LoadCameraConfig();
  ImuPublisher::Config LoadImuConfig();
  bool LoadRateMetricsEnabled();
  uint32_t LoadRateLogPeriodMs();
  std::string LoadRateRunId();

  bool enable_camera_ = true;
  bool enable_imu_ = true;
  bool rate_metrics_enabled_ = false;
  uint32_t rate_log_period_ms_ = 1000U;
  std::string rate_run_id_;
  SensorTimestampMapper timestamp_mapper_;
  std::unique_ptr<CameraPublisher> camera_publisher_;
  std::unique_ptr<ImuPublisher> imu_publisher_;
};

}  // namespace robobaton_4p_ros2_demo
