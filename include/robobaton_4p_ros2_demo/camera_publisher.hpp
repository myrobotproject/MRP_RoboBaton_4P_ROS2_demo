#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "robobaton_4p_ros2_demo/cam_demo_common.h"
#include "robobaton_4p_ros2_demo/timestamp_mapper.hpp"

namespace robobaton_4p_ros2_demo {

class CameraPublisher {
 public:
  enum class QueuePolicy {
    kBlock,
    kDropNewest,
  };

  struct Config {
    const SensorTimestampMapper* timestamp_mapper = nullptr;
    robobaton_demo::Options options;
    std::size_t queue_capacity = 4;
    QueuePolicy queue_policy = QueuePolicy::kBlock;
    bool publish_camera_info = true;
    std::string image_encoding = "nv12";
    bool publish_compressed_image = true;
    int compressed_jpeg_quality = 80;
    std::string frame_id_prefix = "robobaton_cam";
    bool rate_metrics_enabled = false;
    uint32_t rate_log_period_ms = 1000U;
    std::string rate_run_id;
  };

  // The node owns lifecycle management; this class wraps only X5 camera acquisition and ROS publication resources.
  CameraPublisher(rclcpp::Node* node, Config config);
  ~CameraPublisher();

  CameraPublisher(const CameraPublisher&) = delete;
  CameraPublisher& operator=(const CameraPublisher&) = delete;

  void Start();
  void Stop();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robobaton_4p_ros2_demo
