#pragma once

#include <cstdint>
#include <string>
#include "robobaton_4p_ros2_demo/sc132camera.h"

namespace robobaton_demo {

// Maximum number of cameras supported by the demo.
constexpr int kMaxChannels = 4;
constexpr int kSensorInputWidth = 1088;
constexpr int kSensorInputHeight = 1280;
constexpr int kDefaultFps = 30;
constexpr int kDefaultRotateDegrees = 0;
constexpr int kMountRotateDegrees = 90;
constexpr uint32_t kDefaultCameraMask = (1U << kMaxChannels) - 1U;
constexpr uint64_t kDefaultFrameSetMaxSkewNs = SC132_FRAME_SET_DEFAULT_MAX_SKEW_NS;
constexpr uint32_t kDefaultFrameSetTimeoutMs = 100;
constexpr const char* kDefaultSc132TriggerMode = "software_gpio";

// Runtime parameters used by the ROS camera publication path.
// Input source: ROS parameters or defaults.
// Output use: camera initialization, frame-set synchronization, and trigger-mode configuration.
struct Options {
  uint32_t camera_mask = kDefaultCameraMask;
  int fps = kDefaultFps;
  int rotate_degrees = kDefaultRotateDegrees;
  uint64_t frame_set_max_skew_ns = kDefaultFrameSetMaxSkewNs;
  uint32_t frame_set_timeout_ms = kDefaultFrameSetTimeoutMs;
  std::string trigger_mode = kDefaultSc132TriggerMode;
};

// Count the enabled physical cameras in the mask.
// Input: camera_mask bits 0 through 3 correspond to cam0 through cam3.
// Output: the number of enabled bits; higher bits are ignored.
int CameraMaskPopCount(uint32_t camera_mask);

// Check whether a physical camera is enabled by the current mask.
// Input: camera_id from 0 through 3.
// Output: true means a ROS publication worker should start for that camera.
bool CameraMaskContains(uint32_t camera_mask, int camera_id);

// Convert the public rotation angle to the lower-level camera output rotation.
// Input: options.rotate_degrees is the user-facing rotation relative to an upright image.
// Output: the actual rotation passed to libsc132.
int InternalRotateDegrees(const Options& options);

// Return the producer frame width for the actual rotation passed to libsc132; this is a pure calculation.
uint32_t Sc132OutputWidth(const Options& options);
// Return the producer frame height for the actual rotation passed to libsc132; this is a pure calculation.
uint32_t Sc132OutputHeight(const Options& options);

}  // namespace robobaton_demo
