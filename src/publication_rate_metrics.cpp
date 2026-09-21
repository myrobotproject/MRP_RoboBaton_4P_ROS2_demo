#include "robobaton_4p_ros2_demo/publication_rate_metrics.hpp"

#include <limits>

namespace robobaton_4p_ros2_demo {
namespace {

// Adjacent snapshots may cross lifecycles or contain damaged evidence; saturate counter regressions at zero to prevent unsigned underflow from fabricating huge event counts.
uint64_t SaturatingSubtract(uint64_t after, uint64_t before) noexcept {
  return after >= before ? after - before : 0U;
}

// Compute the expected successor using the producer's actual sequence width; wrapping from the maximum value to zero is valid and is not a gap or regression.
uint64_t ExpectedNext(uint64_t sequence, SequenceWidth width) noexcept {
  if (width == SequenceWidth::k32Bit) {
    return static_cast<uint32_t>(static_cast<uint32_t>(sequence) + 1U);
  }
  return sequence == std::numeric_limits<uint64_t>::max() ? 0U : sequence + 1U;
}

// Compare SC132 frame IDs using their 32-bit contract; truncate higher bits before entering shared state so caller extensions cannot affect wrap detection.
uint64_t NormalizeSequence(uint64_t sequence, SequenceWidth width) noexcept {
  return width == SequenceWidth::k32Bit ? static_cast<uint32_t>(sequence) : sequence;
}

}  // namespace

PublicationRateMetrics::PublicationRateMetrics(SequenceWidth sequence_width) noexcept
    : sequence_width_(sequence_width) {}

// Publish the first non-zero monotonic timestamp once; atomic CAS prevents competing camera callbacks from overwriting the measurement start.
void PublicationRateMetrics::SetFirstMonotonic(uint64_t monotonic_ns) noexcept {
  if (monotonic_ns == 0U) {
    return;
  }
  uint64_t expected = 0U;
  (void)first_monotonic_ns_.compare_exchange_strong(
      expected, monotonic_ns, std::memory_order_release, std::memory_order_relaxed);
}

// Update timestamp and sequence history under the same source mutex so duplicate and regression checks use one arrival order.
void PublicationRateMetrics::RecordTimestampLocked(uint64_t timestamp_ns) noexcept {
  // Count equality as a duplicate; only a strictly smaller value is a regression. This layer does not infer drops from normal jitter or large gaps.
  if (timestamp_initialized_) {
    if (timestamp_ns == last_timestamp_ns_) {
      timestamp_duplicate_count_.fetch_add(1U, std::memory_order_relaxed);
    } else if (timestamp_ns < last_timestamp_ns_) {
      timestamp_regression_count_.fetch_add(1U, std::memory_order_relaxed);
    }
  }
  timestamp_initialized_ = true;
  last_timestamp_ns_ = timestamp_ns;
}

// Record producer input with its real sequence; counters use an atomic hot path, while cross-sample state is protected by a short critical section.
void PublicationRateMetrics::RecordSource(uint64_t sequence, uint64_t timestamp_ns,
                                          uint64_t monotonic_ns) noexcept {
  SetFirstMonotonic(monotonic_ns);
  source_count_.fetch_add(1U, std::memory_order_relaxed);
  // The lock covers only the last sequence/timestamp state; publish and drop counters are independent atomics so ROS publication and acquisition do not block each other.
  std::lock_guard<std::mutex> lock(source_mutex_);
  const uint64_t normalized = NormalizeSequence(sequence, sequence_width_);
  if (sequence_width_ != SequenceWidth::kNone && sequence_initialized_) {
    const uint64_t expected = ExpectedNext(last_sequence_, sequence_width_);
    // A normalized 32-bit sequence becomes smaller when it wraps; treat it as a forward wrap only when the previous value was in the upper half and the new value is in the lower half.
    if (normalized == last_sequence_) {
      sequence_duplicate_count_.fetch_add(1U, std::memory_order_relaxed);
    } else if (normalized != expected) {
      const bool wrapped_forward =
          sequence_width_ == SequenceWidth::k32Bit && last_sequence_ > UINT32_MAX / 2U &&
          normalized < UINT32_MAX / 2U;
      // Count one gap for an unexpected forward successor; do not estimate missing frames here. A backward move is a regression.
      if (normalized > expected || wrapped_forward) {
        sequence_gap_count_.fetch_add(1U, std::memory_order_relaxed);
      } else {
        sequence_regression_count_.fetch_add(1U, std::memory_order_relaxed);
      }
    }
  }
  sequence_initialized_ = sequence_width_ != SequenceWidth::kNone;
  last_sequence_ = normalized;
  RecordTimestampLocked(timestamp_ns);
}

// The IMU C ABI has no sequence; reuse only the same timestamp-order lock and do not invent an unauditable sequence in the ROS layer.
void PublicationRateMetrics::RecordSourceWithoutSequence(uint64_t timestamp_ns,
                                                         uint64_t monotonic_ns) noexcept {
  SetFirstMonotonic(monotonic_ns);
  source_count_.fetch_add(1U, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(source_mutex_);
  RecordTimestampLocked(timestamp_ns);
}

// Record call latency after successful publication on the main ROS topic; raise the maximum atomically with CAS without adding a mutex or allocation to the hot path.
void PublicationRateMetrics::RecordPublishSuccess(uint64_t latency_ns) noexcept {
  publish_count_.fetch_add(1U, std::memory_order_relaxed);
  publish_latency_count_.fetch_add(1U, std::memory_order_relaxed);
  publish_latency_sum_ns_.fetch_add(latency_ns, std::memory_order_relaxed);

  // Concurrent camera workers may only increase the maximum; after a failed CAS, reuse the refreshed expected value until no update is needed or the update succeeds.
  uint64_t expected = publish_latency_max_ns_.load(std::memory_order_relaxed);
  while (latency_ns > expected &&
         !publish_latency_max_ns_.compare_exchange_weak(
             expected, latency_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {
  }
}

// CameraInfo and Temperature are auxiliary topics; count them separately so they do not double the main Image or IMU rate.
void PublicationRateMetrics::RecordAuxiliaryPublishSuccess() noexcept {
  auxiliary_publish_count_.fetch_add(1U, std::memory_order_relaxed);
}

void PublicationRateMetrics::RecordDrop() noexcept {
  drop_count_.fetch_add(1U, std::memory_order_relaxed);
}

void PublicationRateMetrics::RecordPublishFailure() noexcept {
  publish_failure_count_.fetch_add(1U, std::memory_order_relaxed);
}

// A snapshot is a lock-free, approximately consistent observation; each cumulative atomic is auditable, and the classifier compares interval deltas without requiring all fields to come from one CPU instruction instant.
PublicationRateSnapshot PublicationRateMetrics::Snapshot(uint64_t monotonic_ns) const noexcept {
  PublicationRateSnapshot value;
  // The acquire load pairs with release publication of the first timestamp; relaxed counter updates still keep each field monotonic and free of data races.
  value.source_count = source_count_.load(std::memory_order_acquire);
  value.publish_count = publish_count_.load(std::memory_order_acquire);
  value.auxiliary_publish_count =
      auxiliary_publish_count_.load(std::memory_order_acquire);
  value.drop_count = drop_count_.load(std::memory_order_acquire);
  value.publish_failure_count = publish_failure_count_.load(std::memory_order_acquire);
  value.sequence_gap_count = sequence_gap_count_.load(std::memory_order_acquire);
  value.sequence_duplicate_count =
      sequence_duplicate_count_.load(std::memory_order_acquire);
  value.sequence_regression_count =
      sequence_regression_count_.load(std::memory_order_acquire);
  value.timestamp_duplicate_count =
      timestamp_duplicate_count_.load(std::memory_order_acquire);
  value.timestamp_regression_count =
      timestamp_regression_count_.load(std::memory_order_acquire);
  value.publish_latency_count = publish_latency_count_.load(std::memory_order_acquire);
  value.publish_latency_sum_ns = publish_latency_sum_ns_.load(std::memory_order_acquire);
  value.publish_latency_max_ns = publish_latency_max_ns_.load(std::memory_order_acquire);
  value.interval_start_monotonic_ns =
      first_monotonic_ns_.load(std::memory_order_acquire);
  value.interval_end_monotonic_ns = monotonic_ns;
  return value;
}

// Convert two cumulative snapshots into interval data; saturate every counter subtraction and use the end of before through the end of after as the time bounds.
PublicationRateSnapshot PublicationRateMetrics::Delta(
    const PublicationRateSnapshot& before,
    const PublicationRateSnapshot& after) noexcept {
  PublicationRateSnapshot value;
  // Preserve the original auditable meaning of each field; keep sequence and timestamp anomalies separate so the classifier can identify the relevant producer contract.
  value.source_count = SaturatingSubtract(after.source_count, before.source_count);
  value.publish_count = SaturatingSubtract(after.publish_count, before.publish_count);
  value.auxiliary_publish_count =
      SaturatingSubtract(after.auxiliary_publish_count, before.auxiliary_publish_count);
  value.drop_count = SaturatingSubtract(after.drop_count, before.drop_count);
  value.publish_failure_count =
      SaturatingSubtract(after.publish_failure_count, before.publish_failure_count);
  value.sequence_gap_count =
      SaturatingSubtract(after.sequence_gap_count, before.sequence_gap_count);
  value.sequence_duplicate_count =
      SaturatingSubtract(after.sequence_duplicate_count, before.sequence_duplicate_count);
  value.sequence_regression_count =
      SaturatingSubtract(after.sequence_regression_count, before.sequence_regression_count);
  value.timestamp_duplicate_count =
      SaturatingSubtract(after.timestamp_duplicate_count, before.timestamp_duplicate_count);
  value.timestamp_regression_count =
      SaturatingSubtract(after.timestamp_regression_count, before.timestamp_regression_count);
  value.publish_latency_count =
      SaturatingSubtract(after.publish_latency_count, before.publish_latency_count);
  value.publish_latency_sum_ns =
      SaturatingSubtract(after.publish_latency_sum_ns, before.publish_latency_sum_ns);
  value.publish_latency_max_ns = after.publish_latency_max_ns;
  // Preserve the actual timestamps even when after precedes before; later window validation can mark the interval inconclusive instead of fabricating a valid one.
  value.interval_start_monotonic_ns = before.interval_end_monotonic_ns;
  value.interval_end_monotonic_ns = after.interval_end_monotonic_ns;
  return value;
}

}  // namespace robobaton_4p_ros2_demo
