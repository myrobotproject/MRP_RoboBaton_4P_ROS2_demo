# RoboBaton 4P ROS2 Demo

## 中文使用说明

### 1. 简介

`robobaton_4p_ros2_demo` 是面向 RoboBaton 4P 的 ROS2 `ament_cmake` 示例包，提供：

- SC132 四目相机的 NV12 图像发布；
- 可选的硬件 JPEG 压缩图像发布；
- ICM-42688 IMU 数据发布；
- IMU 接收频率监视器；
- 默认的四路相机和 IMU 启动配置；
- 面向 X5 目标板的交叉编译和 merged install 产物。

当前仓库版本由 [`VERSION`](VERSION) 定义，当前为 `1.3.1`。

本 README 是本仓库的独立用户文档。构建、安装、启动、参数和话题说明均以本仓库当前内容为准。

### 2. 运行前提

运行时需要：

- RoboBaton 4P 硬件及其 SC132 相机、ICM-42688 IMU；
- X5 目标板上的 ROS 2 Humble 运行环境；
- 本包完整的 merged install 目录；
- 可用的 Fast DDS 共享内存环境。安装后的环境脚本会自动加载本包提供的 Fast DDS 配置。

从源码构建需要：

- 主机端 ROS 2 Humble，且 `/opt/ros/humble/setup.bash` 可用；
- `colcon` 和 Python `ament_package`；
- C++17 编译环境；
- X5 交叉编译包、aarch64 工具链、目标侧 ROS 2 资源和 X5 多媒体库；
- 仓库内 `lib/` 目录中的 SC132 和 ICM-42688 ABI-v2 运行库。

通过 `X5_CROSS_ROOT` 或 `--cross-root` 指定 X5 交叉编译根目录。交叉编译根目录应包含：

```text
scripts/setup_x5_cross_env.sh
toolchain/aarch64_x5_host_toolchain.cmake
```

### 3. 仓库内容

```text
config/robobaton_sensors.yaml       默认 ROS2 参数
config/fastdds/robobaton_shm.xml   Fast DDS 共享内存配置
launch/robobaton_sensors.launch.py 默认启动文件
script/build_x5_ros2.sh            X5 交叉编译脚本
script/robobaton_ros2_env.bash     目标板运行环境脚本
script/verify_install.py           merged install 检查器
include/                           公开头文件
src/                               ROS2 节点和发布器实现
lib/                               随包提供的 ABI-v2 producer 库
```

主要可执行文件和插件：

| 名称 | 用途 |
| --- | --- |
| `robobaton_sensors_node` | 发布相机、IMU 和温度话题 |
| `robobaton_imu_rate_monitor` | 统计 `/robobaton/imu/data` 的实际接收频率 |
| `robobaton_4p_ros2_demo/compressed_pub` | 将 NV12 图像转换为硬件 JPEG compressed transport |

### 4. 构建

在仓库根目录执行：

```bash
export X5_CROSS_ROOT="/path/to/x5/cross_compile/new"
set +u
source /opt/ros/humble/setup.bash
set -u
script/build_x5_ros2.sh --clean --cross-root "$X5_CROSS_ROOT"
```

默认构建输出为：

```text
1.ros2_build/build
1.ros2_build/install
1.ros2_build/log
```

脚本会检查仓库内 `lib/` 的 producer 库、主机 ROS2 Python 环境和 `colcon`，完成构建和 merged install，并自动生成 `abi_manifest.sha256` 后运行 `script/verify_install.py`。

如果需要单独检查已有安装目录：

```bash
python3 script/verify_install.py 1.ros2_build/install
```

`--clean` 只清理构建脚本指定的 build、install、log 输出目录，不删除源代码和仓库内的 `lib/`。

### 5. 部署到目标板

将完整的 merged install 目录复制到目标板。例如：

```bash
rsync -a 1.ros2_build/install/ user@target:/root/ros2_demo/install/
```

目标板上的安装路径可以不同；下面以 `/root/ros2_demo/install` 为例。部署后加载运行环境：

```bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
```

该脚本默认加载 `/opt/ros/humble/setup.bash`，并设置 merged install overlay、本包 Fast DDS 共享内存配置、运行库搜索路径和 `RCUTILS_LOGGING_BUFFERED_STREAM=0`。

如果 ROS2 underlay 不在默认位置：

```bash
export ROBOBATON_ROS_UNDERLAY=/path/to/ros2/setup.bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
```

环境脚本也可以直接作为命令包装器使用：

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash --check
/root/ros2_demo/install/robobaton_ros2_env.bash --list-topics
```

确认所有本包进程已退出后，可以处理 ROS2 daemon 或共享内存中的残留信息：

```bash
/root/ros2_demo/install/robobaton_ros2_env.bash --restart-daemon
/root/ros2_demo/install/robobaton_ros2_env.bash --clean-shm
```

`--clean-shm` 会拒绝在仍有 ROS2 运行进程时清理共享内存文件。

### 6. 启动和停止

启动默认配置，即四路相机和 IMU：

```bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
ros2 launch robobaton_4p_ros2_demo robobaton_sensors.launch.py
```

默认配置文件为 [`config/robobaton_sensors.yaml`](config/robobaton_sensors.yaml)，安装后位于：

```text
<install-prefix>/share/robobaton_4p_ros2_demo/config/robobaton_sensors.yaml
```

只运行 IMU：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=false -p enable_imu:=true
```

只运行相机：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  -p enable_camera:=true -p enable_imu:=false
```

使用自定义参数文件：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_sensors_node --ros-args \
  --params-file /path/to/robobaton_sensors.yaml
```

停止节点时使用 `Ctrl-C`。相机或 IMU 启动失败、发布异常或底层生命周期失败会让节点退出，以避免继续提供不完整的数据流。

### 7. 版本查询

版本查询不会初始化 ROS2 graph、相机或 IMU：

```bash
/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_sensors_node --version
/root/ros2_demo/install/lib/robobaton_4p_ros2_demo/robobaton_imu_rate_monitor --version
```

传感器节点会同时打印 ROS2 demo 版本、`libicm42688` 产品和 ABI 版本，以及 `libsc132` 产品和 ABI 版本。

### 8. ROS2 话题

只有启用的相机才会创建对应话题。`camera.camera_mask` 的 bit0 到 bit3 分别对应 `cam0` 到 `cam3`。

| 话题 | 消息类型 | 说明 |
| --- | --- | --- |
| `/robobaton/cam0/image_raw` 到 `/robobaton/cam3/image_raw` | `sensor_msgs/msg/Image` | NV12 原始图像 |
| `/robobaton/cam0/image_raw/compressed` 到 `/robobaton/cam3/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | X5 硬件 JPEG 图像；可通过参数关闭 |
| `/robobaton/cam0/camera_info` 到 `/robobaton/cam3/camera_info` | `sensor_msgs/msg/CameraInfo` | 当前图像宽高；默认发布，不包含标定参数 |
| `/robobaton/imu/data` | `sensor_msgs/msg/Imu` | 加速度、角速度和 IMU 样本 |
| `/robobaton/imu/temperature` | `sensor_msgs/msg/Temperature` | IMU 温度；可通过参数关闭 |

相机话题说明：

- 默认图像尺寸为 `1280 x 1088`，图像编码为 `nv12`；
- NV12 数据按 Y 平面后接 UV 平面存放，`step` 使用底层帧 stride；
- 默认 frame id 为 `robobaton_cam0_optical_frame` 到 `robobaton_cam3_optical_frame`；
- `CameraInfo` 只填写当前帧的 `width` 和 `height`，不会提供内参、畸变参数或外参；
- compressed 消息的 `format` 为 `nv12; jpeg compressed bgr8`。

IMU 话题说明：

- `angular_velocity` 单位为 rad/s；
- `linear_acceleration` 单位为 m/s²；
- 默认 frame id 为 `robobaton_imu_link`；
- 节点不提供可用的 orientation，`orientation_covariance[0]` 设置为 `-1`；
- 相机和 IMU 都发布 ROS 时间戳；相机在 `software_gpio` 模式下使用 GPIO417 触发时间，IMU 使用传感器时间戳。

查询节点和话题：

```bash
ros2 node list --no-daemon
ros2 topic list --no-daemon
ros2 topic info /robobaton/imu/data --verbose
ros2 topic echo /robobaton/imu/data --once
```

### 9. 参数

所有参数都可以写入 [`config/robobaton_sensors.yaml`](config/robobaton_sensors.yaml)，也可以通过 `--ros-args -p name:=value` 覆盖。

#### 通用参数

| 参数 | 默认值 | 取值和说明 |
| --- | --- | --- |
| `enable_camera` | `true` | 是否启动相机 |
| `enable_imu` | `true` | 是否启动 IMU |
| `diagnostics.rate_metrics_enabled` | `false` | 是否输出发布频率诊断日志 |
| `diagnostics.rate_log_period_ms` | `1000` | 诊断日志周期，启用诊断时必须大于 0 |
| `diagnostics.rate_run_id` | `""` | 诊断运行标识，只允许 ASCII 字母、数字和 `-_.:` |

#### 相机参数

| 参数 | 默认值 | 取值和说明 |
| --- | --- | --- |
| `camera.camera_mask` | `15` | `1`=cam0，`2`=cam1，`4`=cam2，`8`=cam3，`15`=四路；仅支持这些值 |
| `camera.fps` | `30` | 支持 `25/30/40/50/60` |
| `camera.rotate_degrees` | `0` | 支持 `0/90/180/270`；`180` 仅支持 `30fps` |
| `camera.frame_set_max_skew_ns` | `10000000` | 四路帧组最大时间差，单位 ns，必须大于 0 |
| `camera.frame_set_timeout_ms` | `100` | 等待帧组的超时，单位 ms，必须大于 0 |
| `camera.queue_capacity` | `4` | 相机发布队列容量，必须大于 0 |
| `camera.queue_policy` | `block` | `block` 表示队列满时等待；`drop_newest` 表示丢弃新到帧 |
| `camera.publish_camera_info` | `true` | 是否发布 `CameraInfo` |
| `camera.image_encoding` | `nv12` | 当前只支持 `nv12` |
| `camera.publish_compressed_image` | `true` | 是否发布 compressed JPEG 话题 |
| `camera.compressed_jpeg_quality` | `80` | JPEG 质量，范围 `1` 到 `100` |
| `camera.frame_id_prefix` | `robobaton_cam` | 相机 frame id 前缀 |
| `camera.trigger_mode` | `software_gpio` | 支持 `software_gpio` 或 `none`；`software_gpio` 使用 GPIO417 触发路径 |

相机只支持单路或四路组合，不支持任意两路或三路组合。单路模式会根据帧率自动选择对应的 SC132 sensor profile；如果进程环境中已经设置 `SC132_SENSOR_PROFILE`，则保留已有设置。

#### IMU 参数

| 参数 | 默认值 | 取值和说明 |
| --- | --- | --- |
| `imu.sample_rate_hz` | `1000` | 支持 `25/50/100/200/500/1000/2000` |
| `imu.read_mode` | `sensor_timestamp_fifo` | 当前只支持 `sensor_timestamp_fifo` |
| `imu.fifo_watermark_samples` | `1` | 当前只支持 `1` |
| `imu.frame_id` | `robobaton_imu_link` | IMU 消息 frame id |
| `imu.publish_temperature` | `true` | 是否发布 IMU 温度话题 |

### 10. IMU 频率监视器

启动监视器：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor
```

默认监视 `/robobaton/imu/data`，每秒输出一次接收频率。参数如下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `topic` | `/robobaton/imu/data` | 必须是绝对 ROS 话题名，且不能包含空白字符 |
| `report_period_ms` | `1000` | 输出周期，必须大于 0 |
| `qos_depth` | `100` | 订阅队列深度，必须大于 0 |

例如，以 500 ms 周期监视指定话题：

```bash
ros2 run robobaton_4p_ros2_demo robobaton_imu_rate_monitor --ros-args \
  -p topic:=/robobaton/imu/data \
  -p report_period_ms:=500
```

### 11. 支持边界

- 相机支持 `25/30/40/50/60fps`，默认 `30fps`；
- 相机只支持单路或完整四路，不支持 2/3 路组合；
- `camera.rotate_degrees=180` 只支持 `30fps`；
- 图像原始格式为 NV12，compressed 路径使用 X5 硬件 JPEG；
- 本包不提供 RTSP；
- IMU 使用 sensor-timestamp FIFO，采样率支持 `25/50/100/200/500/1000/2000Hz`；
- 默认相机触发模式为 `software_gpio`；
- 本包不提供 TF、外参、相机标定或可用的 IMU orientation；
- `CameraInfo` 不包含标定矩阵；
- 相机和 IMU 采集依赖目标板上的对应硬件、驱动和 vendor 运行库。

### 12. 许可证

本仓库作者编写的源代码、脚本、配置文件和文档使用 Apache-2.0 许可证。`lib/` 目录下的预编译 vendor 共享库不属于 Apache-2.0 授权范围，其分发和使用边界见 [`LICENSE_SCOPE.md`](LICENSE_SCOPE.md)。
