# RoboBaton 4P ROS2 Demo

## User Guide

### 1. Overview

`robobaton_4p_ros2_demo` is a ROS 2 `ament_cmake` demo package for RoboBaton 4P. It provides:

- NV12 image publication from up to four SC132 cameras;
- optional hardware JPEG compressed image publication;
- ICM-42688 IMU publication;
- an IMU receive-rate monitor;
- a default launch configuration for four cameras and the IMU;
- X5 cross-compilation and merged-install output.

The package version is defined by [`VERSION`](VERSION). The current version is `1.3.1`.

This document covers how to build and use the package from this repository. The instructions match the current repository contents.

### 2. Requirements

Runtime requirements:

- RoboBaton 4P hardware with SC132 cameras and an ICM-42688 IMU;
- ROS 2 Humble on the X5 target board;
- a complete merged install of this package;
- a working Fast DDS shared-memory environment. The installed environment script loads the Fast DDS profile shipped by this package.

Source-build requirements:

- ROS 2 Humble on the host, with `/opt/ros/humble/setup.bash` available;
- `colcon` and the Python `ament_package` module;
- a C++17 build environment;
- the X5 cross-compilation bundle, aarch64 toolchain, target ROS 2 resources, and X5 multimedia libraries;
- the ABI-v2 SC132 and ICM-42688 runtime libraries in this repository's `lib/` directory.

Set `X5_CROSS_ROOT` or pass `--cross-root` to select the X5 cross root. The cross root is expected to provide:

```text
scripts/setup_x5_cross_env.sh
toolchain/aarch64_x5_host_toolchain.cmake
```

### 3. Repository Contents

```text
config/robobaton_sensors.yaml       Default ROS 2 parameters
config/fastdds/robobaton_shm.xml   Fast DDS shared-memory profile
launch/robobaton_sensors.launch.py Default launch file
script/build_x5_ros2.sh            X5 cross-build script
script/robobaton_ros2_env.bash     Target runtime environment script
script/verify_install.py           Merged-install verifier
include/                           Public headers
src/                               ROS 2 node and publisher implementation
lib/                               ABI-v2 producer libraries shipped with the package
```

Main executables and plugin:

| Name | Purpose |
| --- | --- |
| `robobaton_sensors_node` | Publishes camera, IMU, and temperature topics |
| `robobaton_imu_rate_monitor` | Measures the receive rate of `/robobaton/imu/data` |
| `robobaton_4p_ros2_demo/compressed_pub` | Converts NV12 images to hardware-JPEG compressed transport |

### 4. Build

Run the following from the repository root:

```bash
export X5_CROSS_ROOT="/path/to/x5/cross_compile/new"
set +u
source /opt/ros/humble/setup.bash
set -u
script/build_x5_ros2.sh --clean --cross-root "$X5_CROSS_ROOT"
```

The default output directories are:

```text
1.ros2_build/build
1.ros2_build/install
1.ros2_build/log
```

The script checks the producer libraries in `lib/`, the host ROS 2 Python environment, and `colcon`; builds the package and merged install; writes `abi_manifest.sha256`; and runs `script/verify_install.py`.

To verify an existing install separately:

```bash
python3 script/verify_install.py 1.ros2_build/install
```

`--clean` removes only the build, install, and log output directories selected by the build script. It does not remove source files or the repository `lib/` directory.

### 5. Deploy to the Target

Copy the complete merged install to the target. For example:

```bash
rsync -a 1.ros2_build/install/ user@target:/root/ros2_demo/install/
```

The target path is configurable; the commands below use `/root/ros2_demo/install` as an example. Load the runtime environment after deployment:

```bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
```

The script loads `/opt/ros/humble/setup.bash` by default and configures the merged-install overlay, the package Fast DDS shared-memory profile, the package runtime-library search path, and `RCUTILS_LOGGING_BUFFERED_STREAM=0`.

If the ROS 2 underlay is installed elsewhere:

```bash
export ROBOBATON_ROS_UNDERLAY=/path/to/ros2/setup.bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
```

The environment script can also be used as a command wrapper:

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash --check
/root/ros2_demo/install/robobaton_ros2_env.bash --list-topics
```

After confirming that all package processes have exited, stale ROS 2 daemon or shared-memory state can be handled with:

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash --restart-daemon
/root/ros2_demo/install/robobaton_ros2_env.bash --clean-shm
```

`--clean-shm` refuses to remove shared-memory files while ROS 2 runtime processes are still detected.

### 6. Launch and Stop

Launch the default configuration with all four cameras and the IMU:

```bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
ros2 launch robobaton_4p_ros2_demo robobaton_sensors.launch.py
```

The default file is [`config/robobaton_sensors.yaml`](config/robobaton_sensors.yaml). After installation it is located at:

```text
<install-prefix>/share/robobaton_4p_ros2_demo/config/robobaton_sensors.yaml
```

Run only the IMU:

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=false -p enable_imu:=true
```

Run only the cameras:

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=true -p enable_imu:=false
```

Use a custom parameter file:

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  --params-file /path/to/robobaton_sensors.yaml
```

Stop the node with `Ctrl-C`. Camera or IMU startup failures, publication failures, and lower-level lifecycle failures terminate the node instead of leaving an incomplete data stream running.

### 7. Version Information

Version queries do not initialize the ROS 2 graph, cameras, or IMU:

```bash
/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_sensors_node --version
/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_imu_rate_monitor --version
```

The sensor node also prints the ROS 2 demo version, the `libicm42688` product and ABI versions, and the `libsc132` product and ABI versions.

### 8. ROS 2 Topics

Only topics for enabled cameras are created. Bits 0 through 3 of `camera.camera_mask` map to `cam0` through `cam3`.

| Topic | Message type | Description |
| --- | --- | --- |
| `/robobaton/cam0/image_raw` through `/robobaton/cam3/image_raw` | `sensor_msgs/msg/Image` | Raw NV12 image |
| `/robobaton/cam0/image_raw/compressed` through `/robobaton/cam3/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | X5 hardware-JPEG image; configurable |
| `/robobaton/cam0/camera_info` through `/robobaton/cam3/camera_info` | `sensor_msgs/msg/CameraInfo` | Current image width and height; enabled by default, no calibration data |
| `/robobaton/imu/data` | `sensor_msgs/msg/Imu` | IMU acceleration, angular velocity, and measurements |
| `/robobaton/imu/temperature` | `sensor_msgs/msg/Temperature` | IMU temperature; configurable |

Camera topic details:

- The default image size is `1280 x 1088` with encoding `nv12`;
- NV12 data is stored as the Y plane followed by the UV plane, and `step` is the producer stride;
- Default frame IDs are `robobaton_cam0_optical_frame` through `robobaton_cam3_optical_frame`;
- `CameraInfo` contains only the current frame `width` and `height`, without camera intrinsics, distortion, or extrinsics;
- The compressed message format is `nv12; jpeg compressed bgr8`.

IMU topic details:

- `angular_velocity` is in rad/s;
- `linear_acceleration` is in m/s²;
- The default frame ID is `robobaton_imu_link`;
- No usable orientation is provided; `orientation_covariance[0]` is set to `-1`;
- Both camera and IMU publish ROS timestamps; the camera uses GPIO417 trigger timestamps in `software_gpio` mode, while the IMU uses sensor timestamps.

Inspect nodes and topics:

```bash
ros2 node list --no-daemon
ros2 topic list --no-daemon
ros2 topic info /robobaton/imu/data --verbose
ros2 topic echo /robobaton/imu/data --once
```

### 9. Parameters

All parameters can be placed in [`config/robobaton_sensors.yaml`](config/robobaton_sensors.yaml) or overridden with `--ros-args -p name:=value`.

#### General Parameters

| Parameter | Default | Values and description |
| --- | --- | --- |
| `enable_camera` | `true` | Enable the camera publisher |
| `enable_imu` | `true` | Enable the IMU publisher |
| `diagnostics.rate_metrics_enabled` | `false` | Enable publication-rate diagnostic logs |
| `diagnostics.rate_log_period_ms` | `1000` | Diagnostic log period; must be greater than 0 when enabled |
| `diagnostics.rate_run_id` | `""` | Run identifier; ASCII letters, digits, and `-_.:` only |

#### Camera Parameters

| Parameter | Default | Values and description |
| --- | --- | --- |
| `camera.camera_mask` | `15` | `1`=cam0, `2`=cam1, `4`=cam2, `8`=cam3, `15`=all four; only these values are supported |
| `camera.fps` | `30` | `25/30/40/50/60` |
| `camera.rotate_degrees` | `0` | `0/90/180/270`; `180` is supported only at `30fps` |
| `camera.frame_set_max_skew_ns` | `10000000` | Maximum frame-set skew in ns; must be greater than 0 |
| `camera.frame_set_timeout_ms` | `100` | Frame-set timeout in ms; must be greater than 0 |
| `camera.queue_capacity` | `4` | Camera publication queue capacity; must be greater than 0 |
| `camera.queue_policy` | `block` | `block` waits when full; `drop_newest` drops newly arrived frames |
| `camera.publish_camera_info` | `true` | Publish `CameraInfo` |
| `camera.image_encoding` | `nv12` | Only `nv12` is supported |
| `camera.publish_compressed_image` | `true` | Publish compressed JPEG topics |
| `camera.compressed_jpeg_quality` | `80` | JPEG quality from `1` to `100` |
| `camera.frame_id_prefix` | `robobaton_cam` | Camera frame-ID prefix |
| `camera.trigger_mode` | `software_gpio` | `software_gpio` or `none`; `software_gpio` uses the GPIO417 trigger path |

The camera supports either one camera or all four cameras. Arbitrary two-camera and three-camera combinations are not supported. In single-camera mode, an SC132 sensor profile is selected automatically from the frame rate. An existing `SC132_SENSOR_PROFILE` process environment value is preserved.

#### IMU Parameters

| Parameter | Default | Values and description |
| --- | --- | --- |
| `imu.sample_rate_hz` | `1000` | `25/50/100/200/500/1000/2000` |
| `imu.read_mode` | `sensor_timestamp_fifo` | Only `sensor_timestamp_fifo` is supported |
| `imu.fifo_watermark_samples` | `1` | Only `1` is supported |
| `imu.frame_id` | `robobaton_imu_link` | IMU message frame ID |
| `imu.publish_temperature` | `true` | Publish the IMU temperature topic |

### 10. IMU Rate Monitor

Start the monitor:

```bash
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor
```

It monitors `/robobaton/imu/data` by default and reports the receive rate once per second. Its parameters are:

| Parameter | Default | Description |
| --- | --- | --- |
| `topic` | `/robobaton/imu/data` | Must be an absolute ROS topic name without whitespace |
| `report_period_ms` | `1000` | Report period; must be greater than 0 |
| `qos_depth` | `100` | Subscription queue depth; must be greater than 0 |

For example, monitor a selected topic every 500 ms:

```bash
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor --ros-args \
  -p topic:=/robobaton/imu/data \
  -p report_period_ms:=500
```

### 11. Support Boundaries

- Camera frame rates are `25/30/40/50/60fps`, with `30fps` as the default;
- the camera supports one camera or all four cameras, not arbitrary two-camera or three-camera combinations;
- `camera.rotate_degrees=180` is supported only at `30fps`;
- raw images use NV12, and the compressed path uses X5 hardware JPEG;
- this package does not provide RTSP;
- the IMU uses sensor-timestamp FIFO and supports `25/50/100/200/500/1000/2000Hz`;
- the default camera trigger mode is `software_gpio`;
- this package does not provide TF, extrinsics, camera calibration, or usable IMU orientation;
- `CameraInfo` does not contain calibration matrices;
- camera and IMU acquisition depends on the corresponding target-board hardware, drivers, and vendor runtime libraries.

### 12. License

The source code, scripts, configuration files, and documentation authored in this repository are licensed under Apache-2.0. Precompiled vendor shared libraries under `lib/` are not covered by Apache-2.0; see [`LICENSE_SCOPE.md`](LICENSE_SCOPE.md) for their distribution and use scope.
