#include "robobaton_4p_ros2_demo/cam_demo_config.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace robobaton_demo {
namespace {

constexpr const char* kSc132SensorProfileEnv = "SC132_SENSOR_PROFILE";
constexpr const char* kSc132TriggerModeEnv = "SC132_TRIGGER_MODE";
constexpr const char* kSc132Single30FpsProfile =
    "sc132gs_linear_1088x1280_raw10_30fps_1lane";
constexpr const char* kSc132Single60FpsProfile =
    "sc132gs_linear_1088x1280_raw10_60fps_1lane";

}  // namespace

void ValidateCameraOptions(const Options& options) {
  switch (options.camera_mask) {
    case 0x1U:
    case 0x2U:
    case 0x4U:
    case 0x8U:
    case 0xFU:
      break;
    default:
      throw std::invalid_argument("camera.camera_mask supports only 1, 2, 4, 8, or 15");
  }

  switch (options.fps) {
    case 25:
    case 30:
    case 40:
    case 50:
    case 60:
      break;
    default:
      throw std::invalid_argument("camera.fps must be 25, 30, 40, 50, or 60");
  }

  switch (options.rotate_degrees) {
    case 0:
    case 90:
    case 180:
    case 270:
      break;
    default:
      throw std::invalid_argument("camera.rotate_degrees must be 0, 90, 180, or 270");
  }
  if (options.rotate_degrees == 180 && options.fps != 30) {
    throw std::invalid_argument("camera.rotate_degrees=180 is supported only at 30fps");
  }
  if (options.frame_set_timeout_ms == 0U || options.frame_set_max_skew_ns == 0U) {
    throw std::invalid_argument("camera frame-set timeout and max skew must be positive");
  }
  if (options.trigger_mode != "software_gpio" && options.trigger_mode != "none") {
    throw std::invalid_argument("camera.trigger_mode must be software_gpio or none");
  }
}

// Write the trigger mode selected by ROS parameters to the environment variable used by libsc132.
// Input: options.trigger_mode accepts software_gpio or none; V1 validates software_gpio only.
// Side effect: overwrite SC132_TRIGGER_MODE for the current process; software_gpio uses GPIO417.
void ConfigureSc132TriggerMode(const Options& options) {
  // ROS parameters take precedence over the shell environment so the process starts with the explicit configuration.
  if (setenv(kSc132TriggerModeEnv, options.trigger_mode.c_str(), 1) != 0) {
    throw std::runtime_error("set SC132_TRIGGER_MODE failed");
  }
  std::cout << kSc132TriggerModeEnv << "=" << options.trigger_mode
            << " (GPIO417 is used when mode=software_gpio)\n";
}

// Single-camera 30fps uses the 30fps master profile; 25/40/50/60fps use the compatible 60fps base master profile, with libsc132 writing the target VTS.
// An explicit SC132_SENSOR_PROFILE takes precedence and is preserved; automatic selection only changes the current process environment for later libsc132 initialization.
void ConfigureSc132SensorProfile(const Options& options) {
  const char* current_profile = std::getenv(kSc132SensorProfileEnv);
  if (current_profile != nullptr && current_profile[0] != '\0') {
    std::cout << "SC132 sensor profile already configured\n";
    return;
  }

  // A single sensor must use a master profile; 30fps slave-right returns -36 from vflow_start on the current board.
  if (CameraMaskPopCount(options.camera_mask) != 1) {
    return;
  }

  // 25/40/50/60fps are derived from the 60fps base profile; 30fps uses the matching master profile.
  const char* profile = options.fps == 30 ? kSc132Single30FpsProfile
                                          : kSc132Single60FpsProfile;
  if (setenv(kSc132SensorProfileEnv, profile, 1) != 0) {
    throw std::runtime_error("set SC132_SENSOR_PROFILE failed");
  }
  std::cout << "Auto selected single-sensor " << options.fps << "fps profile\n";
}

}  // namespace robobaton_demo
