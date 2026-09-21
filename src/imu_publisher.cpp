#include "robobaton_4p_ros2_demo/imu_publisher.hpp"
#include "robobaton_4p_ros2_demo/cam_demo_common.h"
#include "robobaton_4p_ros2_demo/publication_rate_metrics.hpp"
#include "robobaton_4p_ros2_demo/timestamp_mapper.hpp"
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "robobaton_4p_ros2_demo/icm42688_driver.h"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace robobaton_4p_ros2_demo {
void RecordProcessFailure() noexcept;
}

namespace {

struct ImuCoreConfig {
  uint32_t sample_rate_hz = 1000U;
  uint32_t fifo_watermark_samples = 1U;
  uint32_t read_mode = ICM42688_READ_MODE_SENSOR_TIMESTAMP_FIFO;
  bool rate_metrics_enabled = false;
};

class ImuLifecycleCore {
 public:
  using Sink = std::function<void(const icm42688_sample_t&)>;
  using FailureNotifier = std::function<void()>;

  ImuLifecycleCore() = default;
  ~ImuLifecycleCore() { (void)Stop(); }

  ImuLifecycleCore(const ImuLifecycleCore&) = delete;
  ImuLifecycleCore& operator=(const ImuLifecycleCore&) = delete;

  bool Start(const ImuCoreConfig& config, Sink sink, FailureNotifier notifier) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (start_claimed_ || terminal_) {
      return false;
    }
    start_claimed_ = true;
    sink_ = std::move(sink);
    notifier_ = std::move(notifier);
    metrics_enabled_ = config.rate_metrics_enabled;

    icm42688_config_t producer_config = ICM42688_CONFIG_INIT;
    producer_config.sample_rate_hz = config.sample_rate_hz;
    producer_config.fifo_watermark_samples = config.fifo_watermark_samples;
    producer_config.read_mode = config.read_mode;

    icm42688_handle_t* candidate = nullptr;
    const int create_status = icm42688_create(&producer_config, &candidate);
    if (create_status != ICM42688_STATUS_OK || candidate == nullptr) {
      if (candidate != nullptr) {
        icm42688_destroy(candidate);
      }
      terminal_ = true;
      RecordFailure();
      return false;
    }
    handle_ = candidate;

    if (icm42688_set_callback(handle_, &ImuLifecycleCore::SampleTrampoline, this) !=
        ICM42688_STATUS_OK) {
      admission_.store(false, std::memory_order_release);
      icm42688_destroy(handle_);
      handle_ = nullptr;
      terminal_ = true;
      RecordFailure();
      return false;
    }

    admission_.store(true, std::memory_order_release);
    start_attempted_ = true;
    const int start_status = icm42688_start(handle_);
    if (start_status != ICM42688_STATUS_OK) {
      admission_.store(false, std::memory_order_release);
      const int stop_status = icm42688_stop(handle_);
      if (stop_status != ICM42688_STATUS_OK) {
        stop_failed_.store(true, std::memory_order_release);
      }
      // Even after a partial start failure, the sole owner destroys the non-null C handle.
      icm42688_destroy(handle_);
      handle_ = nullptr;
      terminal_ = true;
      RecordFailure();
      sink_ = {};
      notifier_ = {};
      return false;
    }
    running_ = true;
    return true;
  }

  bool Stop() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (terminal_ && handle_ == nullptr) {
      return !stop_failed_.load(std::memory_order_acquire);
    }
    terminal_ = true;
    admission_.store(false, std::memory_order_release);
    int stop_status = ICM42688_STATUS_OK;
    if (handle_ != nullptr && start_attempted_) {
      // Keep the context until blocking stop returns; even if stop fails, still run the sole destroy finalizer.
      stop_status = icm42688_stop(handle_);
      if (stop_status != ICM42688_STATUS_OK) {
        stop_failed_.store(true, std::memory_order_release);
        RecordFailure();
      }
    }
    if (handle_ != nullptr) {
      icm42688_destroy(handle_);
      handle_ = nullptr;
    }
    running_ = false;
    sink_ = {};
    notifier_ = {};
    return stop_status == ICM42688_STATUS_OK;
  }

  bool failed() const noexcept { return failed_.load(std::memory_order_acquire); }

 private:
  static void SampleTrampoline(const icm42688_sample_t* sample, void* user_data) noexcept {
    auto* owner = static_cast<ImuLifecycleCore*>(user_data);
    if (owner == nullptr || !owner->admission_.load(std::memory_order_acquire)) {
      return;
    }
    try {
      if (sample == nullptr || sample->struct_size != sizeof(*sample)) {
        throw std::runtime_error("invalid ICM sample");
      }
      // When disabled, do not read the clock or touch metrics mutexes/atomics; when enabled, record producer input before the ROS sink.
      if (owner->metrics_enabled_) {
        owner->metrics_.RecordSourceWithoutSequence(sample->sample_timestamp_ns,
                                                    SteadyNowNs());
      }
      owner->sink_(*sample);
    } catch (...) {
      owner->admission_.store(false, std::memory_order_release);
      owner->RecordFailure();
    }
  }

  // Use steady-clock nanoseconds for callbacks and ROS publication so wall-clock or ROS-time changes cannot affect intervals or latency.
  static uint64_t SteadyNowNs() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }

 public:
  // The ROS layer only records publication results and reads snapshots by value; it does not own the C handle or metrics.
  void RecordPublishSuccess(uint64_t latency_ns) noexcept {
    if (metrics_enabled_) {
      metrics_.RecordPublishSuccess(latency_ns);
    }
  }
  // Count failures from both main and auxiliary ROS publications under the same gate, then let the callback preserve the existing fail-closed behavior.
  void RecordPublishFailure() noexcept {
    if (metrics_enabled_) {
      metrics_.RecordPublishFailure();
    }
  }

  // Count the auxiliary Temperature topic separately so it cannot double the main IMU publication rate.
  void RecordAuxiliaryPublishSuccess() noexcept {
    if (metrics_enabled_) {
      metrics_.RecordAuxiliaryPublishSuccess();
    }
  }

  robobaton_4p_ros2_demo::PublicationRateSnapshot Snapshot(uint64_t monotonic_ns) const noexcept {
    return metrics_.Snapshot(monotonic_ns);
  }

 private:
  void RecordFailure() noexcept {
    const bool first = !failed_.exchange(true, std::memory_order_acq_rel);
    if (first && notifier_) {
      try {
        notifier_();
      } catch (...) {
      }
    }
  }

  std::mutex mutex_;
  icm42688_handle_t* handle_ = nullptr;
  Sink sink_;
  FailureNotifier notifier_;
  std::atomic<bool> admission_{false};
  std::atomic<bool> failed_{false};
  std::atomic<bool> stop_failed_{false};
  robobaton_4p_ros2_demo::PublicationRateMetrics metrics_{
      robobaton_4p_ros2_demo::SequenceWidth::kNone};
  bool metrics_enabled_ = false;
  bool start_claimed_ = false;
  bool start_attempted_ = false;
  bool running_ = false;
  bool terminal_ = false;
};

}  // namespace

namespace robobaton_4p_ros2_demo {
namespace {
constexpr char kImuTopic[] = "/robobaton/imu/data";
constexpr char kTemperatureTopic[] = "/robobaton/imu/temperature";
const SensorTimestampMapper& RequireTimestampMapper(const SensorTimestampMapper* mapper) {
  if (mapper == nullptr) {
    throw std::invalid_argument("IMU publisher requires a timestamp mapper");
  }
  return *mapper;
}

}  // namespace

class ImuPublisher::Impl {
 public:
  Impl(rclcpp::Node* node, Config config)
      : node_(node), config_(std::move(config)),
        timestamp_mapper_(RequireTimestampMapper(config_.timestamp_mapper)) {
    if (node_ == nullptr || (config_.rate_metrics_enabled && config_.rate_log_period_ms == 0U)) {
      throw std::invalid_argument("invalid IMU publisher configuration");
    }
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        kImuTopic, rclcpp::SensorDataQoS().keep_last(100));
    if (config_.publish_temperature) {
      temperature_pub_ = node_->create_publisher<sensor_msgs::msg::Temperature>(
          kTemperatureTopic, rclcpp::SensorDataQoS().keep_last(10));
    }
  }

  ~Impl() { Stop(); }

  void Start() {
    ImuCoreConfig core_config;
    core_config.sample_rate_hz = config_.sample_rate_hz;
    core_config.fifo_watermark_samples = config_.fifo_watermark_samples;
    core_config.read_mode = ICM42688_READ_MODE_SENSOR_TIMESTAMP_FIFO;
    core_config.rate_metrics_enabled = config_.rate_metrics_enabled;
    if (!core_.Start(
            core_config,
            [this](const icm42688_sample_t& sample) { PublishSample(sample); },
            [this] {
              robobaton_4p_ros2_demo::RecordProcessFailure();
              RCLCPP_ERROR(node_->get_logger(), "ICM callback failure; shutting down");
              rclcpp::shutdown();
            })) {
      throw std::runtime_error("ICM publisher start failed or restart was rejected");
    }
    started_ = true;
    RCLCPP_INFO(node_->get_logger(), "Started ICM IMU publisher on %s", kImuTopic);
    if (config_.rate_metrics_enabled) {
      previous_snapshot_ = core_.Snapshot(SteadyNowNs());
      const auto period = std::chrono::milliseconds(config_.rate_log_period_ms);
      rate_timer_ = rclcpp::create_wall_timer(
          period, [this] { LogRateMetrics(); }, nullptr, node_->get_node_base_interface().get(),
          node_->get_node_timers_interface().get());
    }
  }

  void Stop() noexcept {
    rate_timer_.reset();
    if (!started_ && !core_.failed()) {
      return;
    }
    if (!core_.Stop()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
      RCLCPP_ERROR(node_->get_logger(), "ICM stop failed; handle was finalized and restart is forbidden");
    }
    if (core_.failed()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
    }
    started_ = false;
  }

 private:
  // The rate timer only reads a metrics snapshot and emits one JSON line; it does not touch the ICM handle or publication state.
  void LogRateMetrics() {
    const auto current = core_.Snapshot(SteadyNowNs());
    const auto delta = robobaton_4p_ros2_demo::PublicationRateMetrics::Delta(
        previous_snapshot_, current);
    previous_snapshot_ = current;
    const uint64_t interval_start = delta.interval_start_monotonic_ns == 0U
                                        ? current.interval_start_monotonic_ns
                                        : delta.interval_start_monotonic_ns;
    RCLCPP_INFO(
        node_->get_logger(),
        "ROB2_RATE {\"schema\":\"robobaton-rate-v1\",\"run_id\":\"%s\",\"kind\":\"imu\",\"interval_start_monotonic_ns\":%llu,\"interval_end_monotonic_ns\":%llu,\"source\":%llu,\"published\":%llu,\"auxiliary_published\":%llu,\"dropped\":%llu,\"publish_failures\":%llu,\"timestamp_duplicates\":%llu,\"timestamp_regressions\":%llu,\"publish_latency_count\":%llu,\"publish_latency_sum_ns\":%llu,\"publish_latency_max_ns\":%llu}",
        config_.rate_run_id.c_str(), static_cast<unsigned long long>(interval_start),
        static_cast<unsigned long long>(delta.interval_end_monotonic_ns),
        static_cast<unsigned long long>(delta.source_count),
        static_cast<unsigned long long>(delta.publish_count),
        static_cast<unsigned long long>(delta.auxiliary_publish_count),
        static_cast<unsigned long long>(delta.drop_count),
        static_cast<unsigned long long>(delta.publish_failure_count),
        static_cast<unsigned long long>(delta.timestamp_duplicate_count),
        static_cast<unsigned long long>(delta.timestamp_regression_count),
        static_cast<unsigned long long>(delta.publish_latency_count),
        static_cast<unsigned long long>(delta.publish_latency_sum_ns),
        static_cast<unsigned long long>(delta.publish_latency_max_ns));
  }

  static uint64_t SteadyNowNs() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }

  // IMU samples are small, so publishing directly from the ICM callback thread avoids waking a thread per sample and losing samples to queue backpressure.
  void PublishSample(const icm42688_sample_t& sample) {
    sensor_msgs::msg::Imu imu;
    AssignStamp(imu.header.stamp,
                ToRosStampParts(
                    timestamp_mapper_.MapMonotonicRawToRealtimeNs(sample.sample_timestamp_ns)));
    imu.header.frame_id = config_.frame_id;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = sample.gyro_rps[0];
    imu.angular_velocity.y = sample.gyro_rps[1];
    imu.angular_velocity.z = sample.gyro_rps[2];
    imu.linear_acceleration.x = sample.accel_mps2[0];
    imu.linear_acceleration.y = sample.accel_mps2[1];
    imu.linear_acceleration.z = sample.accel_mps2[2];
    // Count an exception from either the main IMU or auxiliary Temperature publication before rethrowing it, preserving the callback's fail-closed behavior.
    try {
      if (config_.rate_metrics_enabled) {
        const uint64_t publish_start_ns = SteadyNowNs();
        imu_pub_->publish(imu);
        core_.RecordPublishSuccess(SteadyNowNs() - publish_start_ns);
      } else {
        imu_pub_->publish(imu);
      }
      if (temperature_pub_) {
        sensor_msgs::msg::Temperature temperature;
        temperature.header = imu.header;
        temperature.temperature = sample.temperature_c;
        temperature_pub_->publish(temperature);
        if (config_.rate_metrics_enabled) {
          core_.RecordAuxiliaryPublishSuccess();
        }
      }
    } catch (...) {
      core_.RecordPublishFailure();
      throw;
    }
  }

  rclcpp::Node* node_;
  Config config_;
  const SensorTimestampMapper& timestamp_mapper_;
  ImuLifecycleCore core_;
  bool started_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::TimerBase::SharedPtr rate_timer_;
  robobaton_4p_ros2_demo::PublicationRateSnapshot previous_snapshot_{};
};

ImuPublisher::ImuPublisher(rclcpp::Node* node, Config config)
    : impl_(std::make_unique<Impl>(node, std::move(config))) {}
ImuPublisher::~ImuPublisher() = default;
void ImuPublisher::Start() { impl_->Start(); }
void ImuPublisher::Stop() { impl_->Stop(); }

}  // namespace robobaton_4p_ros2_demo
