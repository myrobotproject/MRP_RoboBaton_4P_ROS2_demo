#include "robobaton_4p_ros2_demo/cam_demo_common.h"

extern "C" {
#include "robobaton_4p_ros2_demo/sc132camera.h"
}

namespace robobaton_demo {

static_assert(kSensorInputWidth == static_cast<int>(SC132_NATIVE_OUTPUT_HEIGHT) &&
                  kSensorInputHeight == static_cast<int>(SC132_NATIVE_OUTPUT_WIDTH),
              "SC132 sensor axes must be the transpose of public delivery dimensions");

// Count valid physical cameras in the mask.
// Input: count only the four valid cam0..cam3 bits and ignore higher bits.
// Output: the number of valid bits, used as the frame-set camera_count.
int CameraMaskPopCount(uint32_t camera_mask) {
  int count = 0;
  for (int camera_id = 0; camera_id < kMaxChannels; ++camera_id) {
    if ((camera_mask & (1U << static_cast<uint32_t>(camera_id))) != 0) {
      ++count;
    }
  }
  return count;
}

// Check whether a physical camera is enabled.
// Input: return false when camera_id is out of range.
// Output: true means this process starts a ROS publication worker for the camera.
bool CameraMaskContains(uint32_t camera_mask, int camera_id) {
  if (camera_id < 0 || camera_id >= kMaxChannels) {
    return false;
  }
  return (camera_mask & (1U << static_cast<uint32_t>(camera_id))) != 0;
}

// Map the user-facing rotation to the installation-compensated output rotation.
// Input: rotate=0 means an upright image to the user; the SC132 mounting direction requires an internal 90-degree clockwise rotation.
// Output: the actual rotation passed to libsc132.
int InternalRotateDegrees(const Options& options) {
  // Hide the sensor's portrait mounting direction; rotate=0 applies the 90-degree installation compensation internally.
  return (options.rotate_degrees + kMountRotateDegrees) % 360;
}


// The ROS frame-set configuration must match the output axes produced by libsc132's internal rotation.
uint32_t Sc132OutputWidth(const Options& options) {
  const int rotation = InternalRotateDegrees(options);
  return rotation == 90 || rotation == 270
             ? static_cast<uint32_t>(kSensorInputHeight)
             : static_cast<uint32_t>(kSensorInputWidth);
}

// Map this together with Sc132OutputWidth so non-default ROS rotation parameters do not fail at startup.
uint32_t Sc132OutputHeight(const Options& options) {
  const int rotation = InternalRotateDegrees(options);
  return rotation == 90 || rotation == 270
             ? static_cast<uint32_t>(kSensorInputWidth)
             : static_cast<uint32_t>(kSensorInputHeight);
}
}  // namespace robobaton_demo
