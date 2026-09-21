#pragma once

#include <cstdint>

namespace robobaton_4p_ros2_demo {

enum class SensorClockId {
  kRealtime,
  kMonotonicRaw,
};

using SensorClockReadFn = int (*)(SensorClockId clock_id, uint64_t* timestamp_ns,
                                  void* user);

struct SensorTimestampMapperSnapshot {
  uint64_t realtime_start_ns = 0U;
  uint64_t monotonic_raw_start_ns = 0U;
  int64_t realtime_minus_monotonic_raw_ns = 0;
};

struct RosStampParts {
  int32_t sec = 0;
  uint32_t nanosec = 0U;
};

class SensorTimestampMapper final {
 public:
  SensorTimestampMapper();
  SensorTimestampMapper(SensorClockReadFn reader, void* user);

  static int ReadSystemClock(SensorClockId clock_id, uint64_t* timestamp_ns,
                             void* user) noexcept;

  uint64_t MapMonotonicRawToRealtimeNs(uint64_t raw_timestamp_ns) const;

  const SensorTimestampMapperSnapshot& snapshot() const noexcept { return snapshot_; }

 private:
  static SensorTimestampMapperSnapshot Capture(SensorClockReadFn reader, void* user);
  static int64_t CheckedOffset(uint64_t realtime_ns, uint64_t monotonic_raw_ns);

  SensorTimestampMapperSnapshot snapshot_{};
};

RosStampParts ToRosStampParts(uint64_t realtime_timestamp_ns);

template <typename RosTimeMessage>
void AssignStamp(RosTimeMessage& stamp, const RosStampParts& parts) noexcept {
  stamp.sec = parts.sec;
  stamp.nanosec = parts.nanosec;
}

}  // namespace robobaton_4p_ros2_demo
