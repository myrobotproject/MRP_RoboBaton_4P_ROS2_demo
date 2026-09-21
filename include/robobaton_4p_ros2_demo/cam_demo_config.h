#pragma once

#include "robobaton_4p_ros2_demo/cam_demo_common.h"

namespace robobaton_demo {

// Validate the ROS camera parameter contract before any hardware or process-environment side effects.
// Input: options; the camera supports one or four cameras, 25/30/40/50/60fps, and the public rotation set.
// Throws: std::invalid_argument when a parameter or combination is unsupported.
void ValidateCameraOptions(const Options& options);

// Configure the libsc132 trigger output mode.
// Input: options.trigger_mode, defaulting to software_gpio.
// Side effect: set the process-local SC132_TRIGGER_MODE environment variable read during libsc132 initialization.
void ConfigureSc132TriggerMode(const Options& options);

// Select a compatible sensor profile for the requested startup combination.
// Input: runtime options.
// Side effect: set SC132_SENSOR_PROFILE in the process environment when needed.
void ConfigureSc132SensorProfile(const Options& options);

}  // namespace robobaton_demo
