# mpu6500_ros2_control

[English](README.md) | [简体中文](README.zh-CN.md)

基于 Linux IIO 子系统的 InvenSense **MPU6500** 六轴 IMU（三轴陀螺 + 三轴
加速度计）`ros2_control` 传感器插件。

每个 IIO 设备对应一个 `hardware_interface::SensorInterface` 组件。驱动不包含
头部/躯干语义，也不直接操作 SPI/I2C；它只读取内核驱动（`inv-mpu6000-spi`）
导出的 IIO sysfs 属性。

## 特性

- 一个插件实例对应一个 `/dev/iio:deviceN`（或 IIO sysfs 目录）。
- 10 个标准状态接口（姿态四元数、角速度、线加速度），外加可选的
  `orientation_valid` 健康标志。
- 使用设备的 `scale` 属性把原始值换算为 SI 单位。
- 支持安装矩阵：可配置车体轴到传感器轴的映射与符号取反。
- 互补滤波四元数：加速度计约束 roll/pitch，陀螺积分 yaw。MPU6500 没有磁力计，
  yaw 会漂移。
- 读取瞬时失败时沿用上一帧；持续失败后向 `ResourceManager` 返回
  `return_type::ERROR`。
- 集成 `common_cpp` 日志后端（spdlog + ROS 桥接），同时支持 colcon、普通
  CMake 和顶层 CLion 聚合构建。
- 支持可选的第二路 IMU（头部 + 身体）。默认只启用**头部**（SPI3 CS0），
  身体 IMU 接 CS1，默认关闭。

## 构建

### colcon

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mpu6500_ros2_control \
  --cmake-args -DCMAKE_CXX_COMPILER=g++-13 -DBUILD_TESTING=ON
colcon test --packages-select mpu6500_ros2_control
```

在 `microduck_rev` 仓库中，工作区根目录自身包含 `CMakeLists.txt`，需要使用
`--base-paths src` 让 colcon 递归发现功能包：

```bash
cd microduck_rev
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select common_cpp mpu6500_ros2_control \
  --cmake-args -DCMAKE_CXX_COMPILER=g++-13
```

`common_cpp` 需要支持 C++23 的标准库（GCC 13+），因此请使用 GCC 13 构建整个
工作区。

### 普通 CMake / CLion

先构建/安装 `common_cpp`（例如 `colcon build --packages-select common_cpp`），
然后：

```bash
source /opt/ros/humble/setup.bash
cmake -S src/mpu6500_ros2_control -B build-mpu6500 \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_PREFIX_PATH="$PWD/install/common_cpp;/opt/ros/humble" \
  -DBUILD_TESTING=ON
cmake --build build-mpu6500 -j
ctest --test-dir build-mpu6500 --output-on-failure
```

> 在带引号的 CMake 参数中请使用 `;` 分隔多个前缀，不要使用 `:`。
> 冒号不是 CMake 列表分隔符，即使路径存在 `find_package(common_cpp)` 也会失败。

使用 CLion 时打开工作区根目录，选择 GCC 13 并加载 ROS 2 环境。根
`CMakeLists.txt` 会设置 `ROS2_WS_SOURCE_BUILD=ON`，并聚合
`colcon_add_subdirectories(...)` 中列出的功能包；本包此时链接构建树中的
`common_cpp` target。`mpu6500_ros2_control` 已加入该列表。若需要构建测试目标，
请在 CLion 的 CMake 选项中添加 `-DBUILD_TESTING=ON`。

## ros2_control 配置

### 单路 IMU（默认：头部，CS0）

```xml
<ros2_control name="head_imu" type="sensor">
  <hardware>
    <plugin>mpu6500_ros2_control/MPU6500SensorInterface</plugin>
    <param name="iio_device">/dev/iio:device0</param>
    <param name="axis_map">x,y,z</param>
    <param name="log.log_level">info</param>
  </hardware>

  <sensor name="head_imu">
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="orientation.w"/>
    <state_interface name="angular_velocity.x"/>
    <state_interface name="angular_velocity.y"/>
    <state_interface name="angular_velocity.z"/>
    <state_interface name="linear_acceleration.x"/>
    <state_interface name="linear_acceleration.y"/>
    <state_interface name="linear_acceleration.z"/>
    <!-- 可选：姿态滤波器已收敛且传感器健康时为 1。 -->
    <state_interface name="orientation_valid"/>
  </sensor>
</ros2_control>
```

只导出 URDF 中声明的接口。`imu_sensor_broadcaster` 需要上面 10 个标准接口；
`orientation_valid` 是可选的，该广播器不会发布它。
控制器可以只声明其中一部分（例如只声明 `angular_velocity.*` 和
`linear_acceleration.*`）。

### 双路 IMU（头部 CS0 + 身体 CS1）

再添加一个组件。插件本身不区分头部/身体，由 URDF 中的名字和 `iio_device`
路径决定：

```xml
<ros2_control name="body_imu" type="sensor">
  <hardware>
    <plugin>mpu6500_ros2_control/MPU6500SensorInterface</plugin>
    <param name="iio_device">/dev/iio:device1</param>
    <param name="axis_map">x,y,z</param>
    <param name="log.log_level">info</param>
  </hardware>
  <sensor name="body_imu">
    <!-- 同样声明完整状态接口；需要时包含 orientation_valid -->
  </sensor>
</ros2_control>
```

> **本工程接线约定：** `head_imu` 接 SPI3 **CS0**，`body_imu` 接 **CS1**。
> `/dev/iio:deviceN` 的编号由内核探测顺序决定，与 CS 编号无关。请先确认：
>
> ```bash
> for d in /sys/bus/iio/devices/iio:device*; do
>   echo "$d -> $(cat "$d/name")"
> done
> ```
>
> 然后把正确的路径传给对应组件。默认只启用头部 IMU（CS0）；需要第二路时，
> 在 URDF 中加入 `body_imu` 组件即可。

### Xacro 辅助宏

`urdf/mpu6500_imu.urdf.xacro` 提供 `mpu6500_sensor`（单个组件）和
`mpu6500_imu_pair`（头部/身体，带使能开关）：

```xml
<xacro:include filename="$(find mpu6500_ros2_control)/urdf/mpu6500_imu.urdf.xacro"/>

<!-- 默认：只启用头部（CS0），身体关闭。 -->
<xacro:mpu6500_imu_pair
    enable_head="true"
    enable_body="false"
    head_iio_device="/dev/iio:device0"
    body_iio_device="/dev/iio:device1"/>

<!-- 同时启用两路： -->
<xacro:mpu6500_imu_pair enable_head="true" enable_body="true"/>
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `iio_device` | — | **必填。** `/dev/iio:deviceN`、`/sys/bus/iio/devices/iio:deviceN` 或兼容的 sysfs 目录。 |
| `expected_name` | 空 | 精确匹配 IIO 设备名。为空时接受 `mpu6500` 或 `mpu6050`。 |
| `verify_name` | `true` | 设备名不匹配时配置失败。设为 `false` 时只打印警告。 |
| `axis_map` | `x,y,z` | 车体轴 → 传感器轴映射，必须是 x/y/z 的排列，例如 `y,-x,z`。 |
| `gyro_scale` | `0` | 每个 LSB 对应的 rad/s。`0` 表示读取 `in_anglvel_<axis>_scale`（或 `in_anglvel_scale`）。 |
| `accel_scale` | `0` | 每个 LSB 对应的 m/s²。`0` 表示读取 `in_accel_<axis>_scale`（或 `in_accel_scale`）。 |
| `sampling_frequency_hz` | `0` | `0` 不修改设备设置；`>0` 尝试写入 `sampling_frequency`。 |
| `strict_sampling_frequency` | `false` | `sampling_frequency` 写入失败时是否判定为配置错误。 |
| `max_read_failures` | `3` | 已有有效样本后，允许连续读取失败的次数；`0` 表示立即报错。 |
| `orientation.accel_gain` | `0.05` | 互补滤波比例校正增益（1/s）。 |
| `orientation.integral_gain` | `0.0` | 陀螺零偏积分增益（1/s²），`0` 关闭。 |
| `orientation.accel_tolerance` | `0.25` | 加速度模长相对 1 g 的最大允许偏差比例，超出时本帧不做加速度校正。 |
| `orientation.valid_samples` | `25` | 启动阶段需要连续多少个加速度模长≈1 g 的样本才把 `orientation_valid` 置 1；50 Hz 下约 0.5 s。 |

### 日志参数

本包使用进程级的 `common_cpp` 日志后端，支持以下硬件参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `log.log_level` | `info` | `trace`、`debug`、`info`、`warn`、`error`、`critical`、`off`。 |
| `log.log_path` | `./log` | 日志目录，文件名 `log_<name>_<timestamp>.log`。 |
| `log.async` | `true` | 是否使用异步 spdlog 队列。 |
| `log.logger_name` | `mpu6500_ros2_control` | logger 名称。 |
| `log.node_name` | `mpu6500_ros2_control` | 日志行中的前缀。 |
| `log.filter.black` | 空 | 逗号分隔的过滤规则（`start-`、`end-`、`inc-`、`equal-`）。 |

> 日志后端在进程内只初始化一次。同一进程中的两个 IMU 组件必须使用完全相同的
> `log.*` 参数（默认值已满足）；若给两个组件配置不同的
> `log.logger_name`/`log.node_name`，第二个组件初始化会失败。

## 导出的状态接口

| 接口 | 单位 | 来源 |
|---|---|---|
| `orientation.x/y/z/w` | — | 互补滤波输出，body → world 四元数。 |
| `angular_velocity.x/y/z` | rad/s | `in_anglvel_*_raw` × scale，车体系。 |
| `linear_acceleration.x/y/z` | m/s² | `in_accel_*_raw` × scale，车体系。 |
| `orientation_valid` | 0/1 | 可选健康标志，见下文。 |

坐标约定遵循 ROS REP-103：x 向前、y 向左、z 向上。传感器静止直立时读数为
`(0, 0, +g)`，导出单位四元数。`orientation` 只由重力和陀螺产生，yaw 会漂移，
不能作为长期航向参考。

### 姿态有效标志

`orientation_valid` 是**健康/就绪标志**，不是“本帧数据新鲜度”。只有 URDF 中
声明该接口时才会导出。

| 值 | 含义 |
|---|---|
| `0` | 启动阶段、cleanup/reset 后，或持续读取失败。姿态不可信。 |
| `1` | 滤波器已初始化，且连续观察到 `orientation.valid_samples` 个加速度模长≈1 g 的样本。 |

细节：

- 启动阶段，只有加速度模长在 `orientation.accel_tolerance` 范围内接近 1 g 时，
  才允许用它初始化姿态。机器人上电时被抬起、放在斜面上或正在运动，都不会把
  线性加速度误当成重力。
- 置 1 后，后续的线性加速度只会跳过当帧的加速度校正，不会清零；否则机器人
  一开始走路就会反复关闭控制器的姿态相关逻辑。
- `max_read_failures` 以内的瞬时读取失败沿用上一帧，不清除已经置 1 的标志；
  超过阈值返回 `ERROR` 时才清零。
- `on_cleanup`、`on_shutdown`、重新 configure 和滤波器 reset 都会清零。
- **该保证不覆盖 yaw。** MPU6500 没有磁力计，即使 `orientation_valid == 1`，
  yaw 仍是相对量并且会漂移。

`duck_control` 应 claim `body_imu/orientation_valid`，在它为 0 时保持当前目标，
不启动策略，跌倒检测也不投票。控制器侧仍可保留 warm-up 计数作为没有该接口时的
兜底。

## 安装矩阵

`axis_map` 把**车体轴**映射到**传感器轴**，必须是 x/y/z 的排列；在轴前加 `-`
表示取反。示例：

| 安装方式 | `axis_map` | 车体 x / y / z 分别取 |
|---|---|---|
| 默认 | `x,y,z` | 传感器 x、y、z |
| 绕 z 轴旋转 90° | `y,-x,z` | 传感器 y、−传感器 x、传感器 z |
| 倒装 | `x,-y,-z` | 传感器 x、−传感器 y、−传感器 z |

陀螺与加速度使用同一套映射。

## 生命周期与失效行为

- `on_init`：解析参数、校验 `<sensor>` 与状态接口，不访问设备。
- `on_configure`：解析 IIO 目录、校验 `name`、打开 6 个 raw 属性、缓存
  `scale`，可选写入 `sampling_frequency`。
- `read`：对已缓存的文件描述符执行 `pread()`，换算单位、应用轴映射、更新状态
  接口与滤波器。
- `on_cleanup` / `on_shutdown`：关闭全部文件描述符。
- 首次读取失败（还没有有效样本）：返回 `ERROR`。
- 瞬时失败：沿用上一帧并返回 `OK`，最多允许 `max_read_failures` 次连续失败；
  下一次失败返回 `ERROR`。读取成功后计数清零。
- `orientation_valid`：初始为 0，只有满足上面的收敛条件后才置 1；持续失败、
  cleanup/重新 configure 和 reset 都会清零。

本插件不导出命令接口，是只读传感器组件。

## IIO 要求与当前开发板状态

需要的内核配置：

```text
CONFIG_INV_MPU6050_IIO=y
CONFIG_INV_MPU6050_SPI=y
```

每个设备需要的 sysfs 属性：

```text
/sys/bus/iio/devices/iio:deviceN/name
/sys/bus/iio/devices/iio:deviceN/in_anglvel_{x,y,z}_raw
/sys/bus/iio/devices/iio:deviceN/in_accel_{x,y,z}_raw
/sys/bus/iio/devices/iio:deviceN/in_anglvel_scale   （或逐通道 scale）
/sys/bus/iio/devices/iio:deviceN/in_accel_scale     （或逐通道 scale）
```

截至 2026-11-03，`lubancat` 开发板已编入驱动且存在设备树节点，但 SPI 探测失败：

```text
inv-mpu6000-spi spi3.0: invalid whoami 0xff expected 0x70 (MPU6500)
```

当前还没有 MPU6500 的 `iio:deviceN`。上板前需要先检查供电、SPI3 接线/CS 和
`spi-max-frequency`（可先降到 1 MHz 验证）。在此之前，单元测试通过模拟 IIO
sysfs 目录（`test/fake_mpu6500.hpp`）覆盖同一套代码路径。

## 使用 fake MPU6500 测试

测试会在临时目录中创建与 IIO sysfs 相同的结构：

```text
fake_mpu6500_<pid>_<n>/
├─ name
├─ in_anglvel_scale
├─ in_accel_scale
├─ in_anglvel_x_raw ... in_anglvel_z_raw
└─ in_accel_x_raw   ... in_accel_z_raw
```

`FakeMpu6500::set_raw_gyro()` / `set_raw_accel()` 会改写这些文件，因此驱动已打开
的文件描述符可以读到新值。运行：

```bash
ctest --test-dir build-mpu6500 --output-on-failure
# 或
colcon test --packages-select mpu6500_ros2_control --event-handlers console_direct+
```

测试覆盖：

- scale 换算、轴映射/符号、设备名校验、缺失属性、损坏数据、
  `sampling_frequency` 写入与失败、关闭重开；
- 滤波初始化、roll 收敛、纯陀螺积分、非法 `dt`；
- `orientation_valid` 的启动收敛、非 1 g 启动样本拒绝、运动加速度下保持、
  持续失败/cleanup 清零；
- `SensorInterface` 生命周期、接口导出、瞬时失败处理、清理后重配，
  以及两个独立组件（头部 + 身体）同时读取；
- 从构建树加载 pluginlib 插件。

## 限制

- 每个周期读取 6 个 sysfs 属性，未使用 IIO buffer。50 Hz 下开销可忽略；
  高速采样应改用 IIO buffer/libiio 后端。
- `scale` 在 configure 时缓存；运行期改变量程需要重新 configure。
- 并非所有内核都允许写 `sampling_frequency`；默认写入失败只告警，不阻断启动。
- IIO 设备编号在重启/探测顺序变化后不保证稳定；请确认 `name` 后按路径配置，
  生产环境建议加 udev 规则。
- 没有磁力计，yaw 不是绝对航向，会漂移。
- 一个插件实例只对应一个 IIO 设备。双 IMU 是两套组件；不要对两路数据做平均，
  它们测量的是不同刚体。
