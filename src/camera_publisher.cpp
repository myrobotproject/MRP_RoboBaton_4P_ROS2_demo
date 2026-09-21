#include "robobaton_4p_ros2_demo/camera_publisher.hpp"
#include "robobaton_4p_ros2_demo/cam_demo_config.h"
#include "robobaton_4p_ros2_demo/timestamp_mapper.hpp"
#include "robobaton_4p_ros2_demo/publication_rate_metrics.hpp"

#include <image_transport/image_transport.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "robobaton_4p_ros2_demo/sc132camera.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace robobaton_4p_ros2_demo {
void RecordProcessFailure() noexcept;
}

namespace {

constexpr std::size_t kMaxCameras = SC132_FRAME_SET_MAX_CAMERAS;

struct CameraCoreConfig {
  uint32_t camera_mask = 1U;
  uint32_t fps = 30U;
  uint32_t rotation = 90U;
  uint32_t width = SC132_NATIVE_OUTPUT_WIDTH;
  uint32_t height = SC132_NATIVE_OUTPUT_HEIGHT;
  uint32_t timeout_ms = 100U;
  uint64_t max_skew_ns = SC132_FRAME_SET_DEFAULT_MAX_SKEW_NS;
  std::size_t queue_capacity = 2U;
  bool drop_newest = false;
  bool rate_metrics_enabled = false;
};

class RetainedFrameJob {
 public:
  RetainedFrameJob() = default;
  RetainedFrameJob(sc132_frame_t* frame, const sc132_frame_info_t& info)
      : frame_(frame), info_(info) {}
  ~RetainedFrameJob() { Reset(); }

  RetainedFrameJob(const RetainedFrameJob&) = delete;
  RetainedFrameJob& operator=(const RetainedFrameJob&) = delete;

  RetainedFrameJob(RetainedFrameJob&& other) noexcept { MoveFrom(other); }
  RetainedFrameJob& operator=(RetainedFrameJob&& other) noexcept {
    if (this != &other) {
      Reset();
      MoveFrom(other);
    }
    return *this;
  }

  const sc132_frame_info_t& info() const { return info_; }
  bool owns_frame() const { return frame_ != nullptr; }

  void Reset() noexcept {
    if (frame_ != nullptr) {
      sc132_frame_release(frame_);
      frame_ = nullptr;
    }
  }

 private:
  void MoveFrom(RetainedFrameJob& other) noexcept {
    frame_ = other.frame_;
    info_ = other.info_;
    other.frame_ = nullptr;
    other.info_ = {};
  }

  sc132_frame_t* frame_ = nullptr;
  sc132_frame_info_t info_{};
};

class FrameQueue {
 public:
  FrameQueue(std::size_t capacity, bool drop_newest)
      : capacity_(capacity), drop_newest_(drop_newest) {}

  FrameQueue(const FrameQueue&) = delete;
  FrameQueue& operator=(const FrameQueue&) = delete;

  bool Push(RetainedFrameJob job) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopped_) {
      return false;
    }
    if (drop_newest_ && queue_.size() >= capacity_) {
      return false;
    }
    not_full_.wait(lock, [this] { return stopped_ || queue_.size() < capacity_; });
    if (stopped_) {
      return false;
    }
    queue_.push_back(std::move(job));
    lock.unlock();
    not_empty_.notify_one();
    return true;
  }

  bool Pop(RetainedFrameJob* output) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this] { return stopped_ || !queue_.empty(); });
    if (stopped_ || queue_.empty()) {
      return false;
    }
    *output = std::move(queue_.front());
    queue_.pop_front();
    lock.unlock();
    not_full_.notify_one();
    return true;
  }

  void CloseAdmission() noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  void StopAndDetach(std::deque<RetainedFrameJob>* detached) noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
      detached->swap(queue_);
    }
    not_empty_.notify_all();
    not_full_.notify_all();
  }
 private:
  const std::size_t capacity_;
  const bool drop_newest_;
  std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::deque<RetainedFrameJob> queue_;
  bool stopped_ = false;
};


class CameraLifecycleCore {
 public:
  using Sink = std::function<void(const RetainedFrameJob&)>;
  using FailureNotifier = std::function<void()>;

  CameraLifecycleCore() : bridge_(new CallbackBridge()) { bridge_->owner.store(this); }
  ~CameraLifecycleCore() = default;

  CameraLifecycleCore(const CameraLifecycleCore&) = delete;
  CameraLifecycleCore& operator=(const CameraLifecycleCore&) = delete;

  bool Start(const CameraCoreConfig& config, Sink sink, FailureNotifier notifier) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (start_claimed_ || terminal_) {
      return false;
    }
    start_claimed_ = true;
    config_ = config;
    sink_ = std::move(sink);
    failure_notifier_ = std::move(notifier);
    if (!ValidateConfig()) {
      RecordFailure();
      (void)CleanupLocked();
      return false;
    }

    try {
      InitializeQueues();
      bridge_->owner.store(this, std::memory_order_release);
      bridge_->accepting.store(true, std::memory_order_release);
      if (sc132_set_fps(config_.fps) != SC132_STATUS_OK ||
          sc132_set_output_rotation(config_.rotation) != SC132_STATUS_OK) {
        throw std::runtime_error("SC132 configuration failed");
      }
      StartWorkers();
      sc132_frame_set_config_t producer_config = SC132_FRAME_SET_CONFIG_INIT;
      producer_config.callback = &CameraLifecycleCore::FrameSetTrampoline;
      producer_config.user_data = bridge_;
      producer_config.camera_count = EnabledCameraCount();
      producer_config.width = config_.width;
      producer_config.height = config_.height;
      producer_config.timeout_ms = config_.timeout_ms;
      producer_config.max_skew_ns = config_.max_skew_ns;
      start_attempted_ = true;
      const int32_t status = sc132_start_frame_set(&producer_config, config_.camera_mask);
      if (status != SC132_STATUS_OK) {
        throw std::runtime_error("SC132 frame-set startup failed");
      }
      running_ = true;
      return true;
    } catch (...) {
      RecordFailure();
      (void)CleanupLocked();
      return false;
    }
  }

  bool Cleanup() noexcept {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    return CleanupLocked();
  }
  // ROS workers submit publication results through one entry point; the core owns per-camera metrics and validates the bounds.
  void RecordPublishSuccess(uint32_t camera_id, uint64_t latency_ns) noexcept {
    if (config_.rate_metrics_enabled && camera_id < kMaxCameras) {
      camera_metrics_[camera_id].RecordPublishSuccess(latency_ns);
    }
  }
  // Failures from main and auxiliary ROS publication share one gate, and the call stack preserves the existing fail-closed behavior.
  void RecordPublishFailure(uint32_t camera_id) noexcept {
    if (config_.rate_metrics_enabled && camera_id < kMaxCameras) {
      camera_metrics_[camera_id].RecordPublishFailure();
    }
  }


  // CameraInfo is an auxiliary topic; count it separately so it does not affect the main Image rate.
  void RecordAuxiliaryPublishSuccess(uint32_t camera_id) noexcept {
    if (config_.rate_metrics_enabled && camera_id < kMaxCameras) {
      camera_metrics_[camera_id].RecordAuxiliaryPublishSuccess();
    }
  }

  // The timer only reads cumulative atomic snapshots; return by value so metrics locks and lifecycle state are not exposed to the ROS layer.
  robobaton_4p_ros2_demo::PublicationRateSnapshot Snapshot(
      uint32_t camera_id, uint64_t monotonic_ns) const noexcept {
    if (camera_id >= kMaxCameras) {
      return {};
    }
    return camera_metrics_[camera_id].Snapshot(monotonic_ns);
  }


  bool failed() const noexcept { return failed_.load(std::memory_order_acquire); }
 private:
  struct CallbackBridge {
    std::atomic<CameraLifecycleCore*> owner{nullptr};
    std::atomic<bool> accepting{false};
  };

  bool ValidateConfig() const noexcept {
    const uint32_t valid_mask = (1U << kMaxCameras) - 1U;
    return config_.camera_mask != 0U && (config_.camera_mask & ~valid_mask) == 0U &&
           config_.queue_capacity > 0U && config_.width > 0U && config_.height > 0U &&
           config_.timeout_ms > 0U && config_.max_skew_ns > 0U;
  }

  uint32_t EnabledCameraCount() const noexcept {
    uint32_t count = 0U;
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (CameraEnabled(camera_id)) {
        ++count;
      }
    }
    return count;
  }

  bool CameraEnabled(uint32_t camera_id) const noexcept {
    return camera_id < kMaxCameras && (config_.camera_mask & (1U << camera_id)) != 0U;
  }

  void InitializeQueues() {
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (CameraEnabled(camera_id)) {
        queues_[camera_id] =
            std::make_unique<FrameQueue>(config_.queue_capacity, config_.drop_newest);
      }
    }
  }

  void StartWorkers() {
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (CameraEnabled(camera_id)) {
        workers_[camera_id] = std::thread(&CameraLifecycleCore::WorkerLoop, this, camera_id);
      }
    }
  }

  static void FrameSetTrampoline(const sc132_frame_set_t* frame_set, void* user_data) noexcept {
    auto* bridge = static_cast<CallbackBridge*>(user_data);
    if (bridge == nullptr || !bridge->accepting.load(std::memory_order_acquire)) {
      return;
    }
    CameraLifecycleCore* owner = bridge->owner.load(std::memory_order_acquire);
    if (owner == nullptr) {
      return;
    }
    try {
      owner->HandleFrameSet(frame_set);
    } catch (...) {
      owner->CloseAdmissionAndRequestStop();
      owner->RecordFailure();
    }
  }

  void HandleFrameSet(const sc132_frame_set_t* frame_set) {
    if (frame_set == nullptr || frame_set->struct_size != sizeof(*frame_set) ||
        frame_set->camera_count != EnabledCameraCount() || frame_set->camera_count == 0U ||
        frame_set->camera_count > kMaxCameras) {
      throw std::runtime_error("invalid frame-set header");
    }
    std::array<bool, kMaxCameras> seen{};
    for (uint32_t index = 0; index < frame_set->camera_count; ++index) {
      const sc132_frame_set_item_t& item = frame_set->items[index];
      const uint32_t camera_id = item.camera_id;
      if (!CameraEnabled(camera_id) || seen[camera_id] || item.frame == nullptr) {
        throw std::runtime_error("invalid frame-set item");
      }
      seen[camera_id] = true;
      sc132_frame_info_t info{};
      info.struct_size = sizeof(info);
      if (sc132_frame_get_info(item.frame, &info) != SC132_STATUS_OK ||
          info.struct_size != sizeof(info) || info.camera_id != camera_id ||
          info.y_data == nullptr || info.uv_data == nullptr || info.y_size == 0U ||
          info.uv_size == 0U || info.width == 0U || info.height == 0U ||
          info.stride == 0U || info.vstride == 0U) {
        throw std::runtime_error("invalid frame metadata");
      }
      // The metric describes producer continuity as seen by the final callback;
      // record the real sequence so frames dropped by the aggregator remain source gaps instead of being hidden by group_id.
      if (config_.rate_metrics_enabled) {
        camera_metrics_[camera_id].RecordSource(
            info.sequence, info.timestamp_ns, SteadyNowNs());

      }
      sc132_frame_t* frame = item.frame;
      const int32_t retain_status = sc132_frame_retain(frame);
      if (retain_status != SC132_STATUS_OK) {
        throw std::runtime_error("frame retain failed");
      }
      RetainedFrameJob job(frame, info);
      if (!queues_[camera_id]->Push(std::move(job))) {
        if (config_.rate_metrics_enabled) {
          camera_metrics_[camera_id].RecordDrop();
        }
        continue;
      }
    }
  }

  void WorkerLoop(uint32_t camera_id) noexcept {
    try {
      RetainedFrameJob job;
      while (queues_[camera_id]->Pop(&job)) {
        sink_(job);
        job.Reset();
      }
    } catch (...) {
      RecordFailure();
      CloseAdmissionAndRequestStop();
    }
  }

  // Use steady-clock nanoseconds from the CLOCK_MONOTONIC time source so ROS or wall-clock adjustments do not affect latency evidence.
  static uint64_t SteadyNowNs() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }


  void RecordFailure() noexcept {
    const bool first = !failed_.exchange(true, std::memory_order_acq_rel);
    if (first && failure_notifier_) {
      try {
        failure_notifier_();
      } catch (...) {
      }
    }
  }

  void CloseAdmissionAndRequestStop() noexcept {
    bridge_->accepting.store(false, std::memory_order_release);
    if (!stop_requested_.exchange(true, std::memory_order_acq_rel)) {
      sc132_request_stop();
    }
    for (auto& queue : queues_) {
      if (queue) {
        queue->CloseAdmission();
      }
    }
  }

  bool CleanupLocked() noexcept {
    if (!start_claimed_) {
      return true;
    }
    if (cleanup_complete_) {
      return cleanup_quiesced_;
    }
    terminal_ = true;
    CloseAdmissionAndRequestStop();

    std::array<std::deque<RetainedFrameJob>, kMaxCameras> detached;
    for (std::size_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (queues_[camera_id]) {
        queues_[camera_id]->StopAndDetach(&detached[camera_id]);
      }
    }
    // Detach inside each queue lock first, then release all retained frames outside the queue locks.
    for (auto& batch : detached) {
      batch.clear();
    }
    try {
      for (auto& worker : workers_) {
        if (worker.joinable()) {
          worker.join();
        }
      }
    } catch (const std::system_error&) {
      cleanup_quiesced_ = false;
      RecordFailure();
      return false;
    }

    // The same owner may call stop twice; the second call confirms idempotence or retries STOPPING cleanup.
    sc132_stop();
    sc132_stop();
    running_ = false;
    cleanup_quiesced_ = true;
    cleanup_complete_ = true;
    sink_ = {};
    failure_notifier_ = {};
    return true;
  }

  CameraCoreConfig config_{};
  CallbackBridge* const bridge_;
  Sink sink_;
  FailureNotifier failure_notifier_;
  std::array<std::unique_ptr<FrameQueue>, kMaxCameras> queues_{};
  std::array<std::thread, kMaxCameras> workers_{};
  std::array<robobaton_4p_ros2_demo::PublicationRateMetrics, kMaxCameras> camera_metrics_{
      robobaton_4p_ros2_demo::PublicationRateMetrics(robobaton_4p_ros2_demo::SequenceWidth::k64Bit),
      robobaton_4p_ros2_demo::PublicationRateMetrics(robobaton_4p_ros2_demo::SequenceWidth::k64Bit),
      robobaton_4p_ros2_demo::PublicationRateMetrics(robobaton_4p_ros2_demo::SequenceWidth::k64Bit),
      robobaton_4p_ros2_demo::PublicationRateMetrics(robobaton_4p_ros2_demo::SequenceWidth::k64Bit)};
  std::mutex lifecycle_mutex_;
  std::atomic<bool> failed_{false};
  std::atomic<bool> stop_requested_{false};
  bool start_claimed_ = false;
  bool start_attempted_ = false;
  bool running_ = false;
  bool terminal_ = false;
  bool cleanup_complete_ = false;
  bool cleanup_quiesced_ = false;
};

}  // namespace

namespace robobaton_4p_ros2_demo {
namespace {

constexpr char kImageTopicSuffix[] = "/image_raw";
constexpr char kCameraInfoTopicSuffix[] = "/camera_info";

bool Sc132TimestampsAreMonotonicRaw(const robobaton_demo::Options& options) noexcept {
  return options.trigger_mode == "software_gpio" || options.trigger_mode == "gpio";
}

const SensorTimestampMapper& RequireTimestampMapper(const SensorTimestampMapper* mapper) {
  if (mapper == nullptr) {
    throw std::invalid_argument("camera publisher requires a timestamp mapper");
  }
  return *mapper;
}

std::string CameraTopic(uint32_t camera_id, const char* suffix) {
  return "/robobaton/cam" + std::to_string(camera_id) + suffix;
}

std::string CameraFrameId(const std::string& prefix, uint32_t camera_id) {
  return prefix + std::to_string(camera_id) + "_optical_frame";
}

std::string ImageTransportParameterBase(rclcpp::Node* node, const std::string& base_topic) {
  std::string expanded_topic = rclcpp::expand_topic_or_service_name(
      base_topic, node->get_name(), node->get_namespace());
  const auto namespace_length = node->get_effective_namespace().length();
  if (expanded_topic.size() >= namespace_length) {
    expanded_topic = expanded_topic.substr(namespace_length);
  }
  std::replace(expanded_topic.begin(), expanded_topic.end(), '/', '.');
  if (!expanded_topic.empty() && expanded_topic.front() == '.') {
    expanded_topic.erase(0, 1);
  }
  return expanded_topic;
}

template <typename T>
void DeclareOrSetImageTransportParameter(rclcpp::Node* node, const std::string& name,
                                         const T& value) {
  if (!node->has_parameter(name)) {
    (void)node->declare_parameter<T>(name, value);
    return;
  }
  const auto result = node->set_parameter(rclcpp::Parameter(name, value));
  if (!result.successful) {
    throw std::invalid_argument("failed to set image_transport parameter " + name + ": " +
                                result.reason);
  }
}

void ConfigureImageTransportPublisher(rclcpp::Node* node, const std::string& base_topic,
                                      bool compressed_enabled, int jpeg_quality) {
  std::vector<std::string> enabled_plugins{"image_transport/raw"};
  if (compressed_enabled) {
    enabled_plugins.emplace_back("robobaton_4p_ros2_demo/compressed");
  }
  const std::string parameter_base = ImageTransportParameterBase(node, base_topic);
  // Mirror image_transport's internal parameter naming so installing another transport plugin does not expose an unintended topic.
  DeclareOrSetImageTransportParameter(node, parameter_base + ".enable_pub_plugins",
                                      enabled_plugins);
  if (compressed_enabled) {
    DeclareOrSetImageTransportParameter(node, parameter_base + ".jpeg_quality",
                                        static_cast<int64_t>(jpeg_quality));
  }
}

CameraLifecycleCore& ProcessCameraCore() {
  // The camera core lives for the process lifetime; once stopped, it cannot be restarted in the same process.
  static CameraLifecycleCore* core = new CameraLifecycleCore();
  return *core;
}

}  // namespace

class CameraPublisher::Impl {
 public:
  Impl(rclcpp::Node* node, Config config)
      : node_(node), config_(std::move(config)),
        timestamp_mapper_(RequireTimestampMapper(config_.timestamp_mapper)),
        core_(ProcessCameraCore()) {
    if (node_ == nullptr || config_.queue_capacity == 0U || config_.image_encoding != "nv12" ||
        config_.compressed_jpeg_quality < 1 || config_.compressed_jpeg_quality > 100 ||
        (config_.rate_metrics_enabled && config_.rate_log_period_ms == 0U)) {
      throw std::invalid_argument("invalid camera publisher configuration");
    }
    auto image_qos = rclcpp::SensorDataQoS().reliable().keep_last(8);
    auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (!robobaton_demo::CameraMaskContains(config_.options.camera_mask,
                                               static_cast<int>(camera_id))) {
        continue;
      }
      const std::string image_topic = CameraTopic(camera_id, kImageTopicSuffix);
      ConfigureImageTransportPublisher(node_, image_topic, config_.publish_compressed_image,
                                       config_.compressed_jpeg_quality);
      image_publishers_[camera_id] = image_transport::create_publisher(
          node_, image_topic, image_qos.get_rmw_qos_profile());
      if (config_.publish_camera_info) {
        camera_info_publishers_[camera_id] =
            node_->create_publisher<sensor_msgs::msg::CameraInfo>(
                CameraTopic(camera_id, kCameraInfoTopicSuffix), info_qos);
      }
    }
  }

  ~Impl() { Stop(); }

  void Start() {
    robobaton_demo::ValidateCameraOptions(config_.options);
    robobaton_demo::ConfigureSc132TriggerMode(config_.options);
    robobaton_demo::ConfigureSc132SensorProfile(config_.options);
    CameraCoreConfig core_config;
    core_config.camera_mask = config_.options.camera_mask;
    core_config.fps = static_cast<uint32_t>(config_.options.fps);
    core_config.rotation =
        static_cast<uint32_t>(robobaton_demo::InternalRotateDegrees(config_.options));
    core_config.width = robobaton_demo::Sc132OutputWidth(config_.options);
    core_config.height = robobaton_demo::Sc132OutputHeight(config_.options);
    core_config.timeout_ms = config_.options.frame_set_timeout_ms;
    core_config.max_skew_ns = config_.options.frame_set_max_skew_ns;
    core_config.queue_capacity = config_.queue_capacity;
    core_config.drop_newest = config_.queue_policy == QueuePolicy::kDropNewest;
    core_config.rate_metrics_enabled = config_.rate_metrics_enabled;
    if (!core_.Start(
            core_config,
            [this](const RetainedFrameJob& job) { PublishFrame(job); },
            [this] {
              robobaton_4p_ros2_demo::RecordProcessFailure();
              RCLCPP_ERROR(node_->get_logger(), "SC132 lifecycle failure; shutting down");
              rclcpp::shutdown();
            })) {
      throw std::runtime_error("SC132 publisher start failed or restart was rejected");
    }
    started_ = true;
    RCLCPP_INFO(node_->get_logger(), "Started SC132 camera publisher mask=0x%X",
                config_.options.camera_mask);
    if (config_.rate_metrics_enabled) {
      previous_snapshots_.fill({});
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
    if (!core_.Cleanup()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
      RCLCPP_FATAL(node_->get_logger(), "SC132 worker join failed; terminating fail-closed");
      std::_Exit(1);
    }
    if (core_.failed()) {
      robobaton_4p_ros2_demo::RecordProcessFailure();
    }
    started_ = false;
  }

 private:
  // Emit fixed-schema single-line JSON directly as node stderr evidence; when disabled, no timer or extra log is created.
  void LogRateMetrics() {
    const uint64_t now_ns = SteadyNowNs();
    for (uint32_t camera_id = 0; camera_id < kMaxCameras; ++camera_id) {
      if (!robobaton_demo::CameraMaskContains(config_.options.camera_mask,
                                               static_cast<int>(camera_id))) {
        continue;
      }
      const auto current = core_.Snapshot(camera_id, now_ns);
      const auto delta = robobaton_4p_ros2_demo::PublicationRateMetrics::Delta(
          previous_snapshots_[camera_id], current);
      previous_snapshots_[camera_id] = current;

      // Start the first window at the first producer sample, then use adjacent timer boundaries; retain raw counts and nanoseconds for host-side recomputation.
      const uint64_t interval_start = delta.interval_start_monotonic_ns == 0U
                                          ? current.interval_start_monotonic_ns
                                          : delta.interval_start_monotonic_ns;
      RCLCPP_INFO(
          node_->get_logger(),
          "ROB2_RATE {\"schema\":\"robobaton-rate-v1\",\"run_id\":\"%s\",\"kind\":\"camera\",\"id\":%u,\"interval_start_monotonic_ns\":%llu,\"interval_end_monotonic_ns\":%llu,\"source\":%llu,\"published\":%llu,\"auxiliary_published\":%llu,\"dropped\":%llu,\"publish_failures\":%llu,\"sequence_gaps\":%llu,\"sequence_duplicates\":%llu,\"sequence_regressions\":%llu,\"timestamp_duplicates\":%llu,\"timestamp_regressions\":%llu,\"publish_latency_count\":%llu,\"publish_latency_sum_ns\":%llu,\"publish_latency_max_ns\":%llu}",
          config_.rate_run_id.c_str(), camera_id,
          static_cast<unsigned long long>(interval_start),
          static_cast<unsigned long long>(delta.interval_end_monotonic_ns),
          static_cast<unsigned long long>(delta.source_count),
          static_cast<unsigned long long>(delta.publish_count),
          static_cast<unsigned long long>(delta.auxiliary_publish_count),
          static_cast<unsigned long long>(delta.drop_count),
          static_cast<unsigned long long>(delta.publish_failure_count),
          static_cast<unsigned long long>(delta.sequence_gap_count),
          static_cast<unsigned long long>(delta.sequence_duplicate_count),
          static_cast<unsigned long long>(delta.sequence_regression_count),
          static_cast<unsigned long long>(delta.timestamp_duplicate_count),
          static_cast<unsigned long long>(delta.timestamp_regression_count),
          static_cast<unsigned long long>(delta.publish_latency_count),
          static_cast<unsigned long long>(delta.publish_latency_sum_ns),
          static_cast<unsigned long long>(delta.publish_latency_max_ns));
    }
  }

  // Measure ROS publication latency with the steady clock, independent of ROS simulation time or wall-clock adjustments.
  static uint64_t SteadyNowNs() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }

  RosStampParts HeaderStampFromFrame(const sc132_frame_info_t& info) const {
    if (Sc132TimestampsAreMonotonicRaw(config_.options)) {
      return ToRosStampParts(
          timestamp_mapper_.MapMonotonicRawToRealtimeNs(info.timestamp_ns));
    }
    return ToRosStampParts(info.timestamp_ns);
  }

  void PublishFrame(const RetainedFrameJob& job) {
    const auto& info = job.info();
    if (!job.owns_frame() || info.y_size > std::numeric_limits<std::size_t>::max() ||
        info.uv_size > std::numeric_limits<std::size_t>::max() ||
        info.y_size > std::numeric_limits<std::size_t>::max() - info.uv_size) {
      throw std::runtime_error("invalid SC132 frame size");
    }
    const uint32_t camera_id = info.camera_id;
    if (camera_id >= kMaxCameras || !image_publishers_[camera_id]) {
      throw std::runtime_error("invalid SC132 camera id");
    }
    sensor_msgs::msg::Image image;
    AssignStamp(image.header.stamp, HeaderStampFromFrame(info));
    image.header.frame_id = CameraFrameId(config_.frame_id_prefix, camera_id);
    image.height = info.height;
    image.width = info.width;
    image.encoding = config_.image_encoding;
    image.is_bigendian = false;
    image.step = info.stride;
    const std::size_t y_size = static_cast<std::size_t>(info.y_size);
    const std::size_t uv_size = static_cast<std::size_t>(info.uv_size);
    image.data.resize(y_size + uv_size);
    std::memcpy(image.data.data(), info.y_data, y_size);
    std::memcpy(image.data.data() + y_size, info.uv_data, uv_size);
    // Count an exception from main Image, compressed Image, or auxiliary CameraInfo publication before rethrowing it, preserving the worker's fail-closed behavior.
    try {
      if (config_.rate_metrics_enabled) {
        const uint64_t publish_start_ns = SteadyNowNs();
        image_publishers_[camera_id].publish(image);
        core_.RecordPublishSuccess(camera_id, SteadyNowNs() - publish_start_ns);
      } else {
        image_publishers_[camera_id].publish(image);
      }
      if (camera_info_publishers_[camera_id]) {
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header = image.header;
        camera_info.width = image.width;
        camera_info.height = image.height;
        camera_info_publishers_[camera_id]->publish(camera_info);
        if (config_.rate_metrics_enabled) {
          core_.RecordAuxiliaryPublishSuccess(camera_id);
        }
      }
    } catch (const std::exception& error) {
      core_.RecordPublishFailure(camera_id);
      RCLCPP_ERROR(node_->get_logger(), "SC132 image publish failed: %s", error.what());
      throw;
    } catch (...) {
      core_.RecordPublishFailure(camera_id);
      RCLCPP_ERROR(node_->get_logger(), "SC132 image publish failed with unknown exception");
      throw;
    }

  }

  rclcpp::Node* node_;
  Config config_;
  const SensorTimestampMapper& timestamp_mapper_;
  CameraLifecycleCore& core_;
  bool started_ = false;
  std::array<image_transport::Publisher, kMaxCameras> image_publishers_{};
  std::array<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr, kMaxCameras>
      camera_info_publishers_{};
  rclcpp::TimerBase::SharedPtr rate_timer_;
  std::array<robobaton_4p_ros2_demo::PublicationRateSnapshot, kMaxCameras>
      previous_snapshots_{};
};

CameraPublisher::CameraPublisher(rclcpp::Node* node, Config config)
    : impl_(std::make_unique<Impl>(node, std::move(config))) {}
CameraPublisher::~CameraPublisher() = default;
void CameraPublisher::Start() { impl_->Start(); }
void CameraPublisher::Stop() { impl_->Stop(); }

}  // namespace robobaton_4p_ros2_demo
