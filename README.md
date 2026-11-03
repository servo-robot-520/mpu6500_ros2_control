# mpu6500_ros2_control

[English](README.md) | [简体中文](README.zh-CN.md)

`ros2_control` sensor interface for the InvenSense **MPU6500** 6-axis IMU
(3-axis gyroscope + 3-axis accelerometer) through the Linux IIO subsystem.

The package exports one `hardware_interface::SensorInterface` component per IIO
device. It does not contain head/trunk semantics and does not open SPI/I2C
directly; it reads the IIO sysfs attributes exposed by the kernel driver
(`inv-mpu6000-spi`).

## Features

- One plugin instance per `/dev/iio:deviceN` (or per IIO sysfs directory).
- 10 exported state interfaces: quaternion orientation, angular velocity and
  linear acceleration.
- Raw samples are converted to SI units using the device's `scale` attributes.
- Configurable body-axis remapping and sign inversion for the mounting matrix.
- Complementary quaternion filter: accelerometer constrains roll/pitch, gyro
  integrates yaw. MPU6500 has no magnetometer, so yaw drifts.
- Transient read failures keep the last sample; persistent failures report
  `return_type::ERROR` to the `ResourceManager`.
- `common_cpp` logging backend (`spdlog` + ROS bridge), compatible with colcon,
  plain CMake and the workspace CLion aggregate build.
- Optional second IMU (head + body). Default configuration enables **head only**
  (SPI3 CS0); the body IMU is CS1 and is disabled by default.

## Package layout

```text
mpu6500_ros2_control/
├─ include/mpu6500_ros2_control/
│  ├─ mpu6500_iio_device.hpp      # IIO sysfs reader (no ROS dependency)
│  ├─ mpu6500_sensor_interface.hpp
│  ├─ orientation_filter.hpp
│  └─ result.hpp
├─ src/
│  ├─ mpu6500_iio_device.cpp
│  ├─ mpu6500_sensor_interface.cpp
│  └─ orientation_filter.cpp
├─ test/
│  ├─ fake_mpu6500.hpp            # temporary sysfs-like fake device
│  ├─ test_iio_device.cpp
│  ├─ test_orientation_filter.cpp
│  ├─ test_sensor_interface.cpp
│  └─ test_plugin_loading.cpp
├─ urdf/mpu6500_imu.urdf.xacro    # single / dual IMU component macros
├─ mpu6500_ros2_control_plugin_description.xml
├─ CMakeLists.txt
└─ package.xml
```

## Build

### colcon

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mpu6500_ros2_control \
  --cmake-args -DCMAKE_CXX_COMPILER=g++-13 -DBUILD_TESTING=ON
colcon test --packages-select mpu6500_ros2_control
```

In the `microduck_rev` repository the workspace root itself contains a
`CMakeLists.txt`, so prefix discovery with `--base-paths src`:

```bash
cd microduck_rev
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select common_cpp mpu6500_ros2_control \
  --cmake-args -DCMAKE_CXX_COMPILER=g++-13
```

`common_cpp` requires a C++23 standard library (GCC 13+), so build the whole
workspace with GCC 13.

### Plain CMake / CLion

`common_cpp` must be installed first (for example with
`colcon build --packages-select common_cpp`). Then:

```bash
source /opt/ros/humble/setup.bash
cmake -S src/mpu6500_ros2_control -B build-mpu6500 \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_PREFIX_PATH="$PWD/install/common_cpp;/opt/ros/humble" \
  -DBUILD_TESTING=ON
cmake --build build-mpu6500 -j
ctest --test-dir build-mpu6500 --output-on-failure
```

> Within a quoted CMake argument use `;` to separate prefixes, not `:`.
> A colon is not a CMake list separator and `find_package(common_cpp)` will
> fail even though the path exists.

For CLion, open the workspace root and select GCC 13 with the sourced ROS 2
environment. The root `CMakeLists.txt` sets `ROS2_WS_SOURCE_BUILD=ON` and adds
the packages listed in `colcon_add_subdirectories(...)`; this package then links
the in-tree `common_cpp` targets. `mpu6500_ros2_control` is part of that list.
Add `-DBUILD_TESTING=ON` to the CLion CMake options to build the tests.

## ros2_control configuration

### Single IMU (default: head, CS0)

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
  </sensor>
</ros2_control>
```

Only interfaces declared here are exported. `imu_sensor_broadcaster` requires
the standard 10 interfaces above; a controller may declare a subset (for
example only `angular_velocity.*` and `linear_acceleration.*`).

### Two IMUs (head CS0 + body CS1)

Add a second component. The plugin itself does not know which one is head or
body; the URDF names and `iio_device` paths define that:

```xml
<ros2_control name="body_imu" type="sensor">
  <hardware>
    <plugin>mpu6500_ros2_control/MPU6500SensorInterface</plugin>
    <param name="iio_device">/dev/iio:device1</param>
    <param name="axis_map">x,y,z</param>
    <param name="log.log_level">info</param>
  </hardware>
  <sensor name="body_imu">
    <!-- same 10 state interfaces -->
  </sensor>
</ros2_control>
```

> **Project wiring:** `head_imu` is on SPI3 **CS0**, `body_imu` on **CS1**.
> The `/dev/iio:deviceN` numbering depends on kernel probe order, not on the CS
> index. Verify with:
>
> ```bash
> for d in /sys/bus/iio/devices/iio:device*; do
>   echo "$d -> $(cat "$d/name")"
> done
> ```
>
> and pass the matching path to each component. The default setup enables only
> the head IMU (CS0); add the `body_imu` component to the URDF to enable the
> second one.

### Xacro helper

`urdf/mpu6500_imu.urdf.xacro` provides `mpu6500_sensor` (one component) and
`mpu6500_imu_pair` (head/body with enable flags):

```xml
<xacro:include filename="$(find mpu6500_ros2_control)/urdf/mpu6500_imu.urdf.xacro"/>

<!-- Default: head only (CS0); body disabled. -->
<xacro:mpu6500_imu_pair
    enable_head="true"
    enable_body="false"
    head_iio_device="/dev/iio:device0"
    body_iio_device="/dev/iio:device1"/>

<!-- Enable both: -->
<xacro:mpu6500_imu_pair enable_head="true" enable_body="true"/>
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `iio_device` | — | **Required.** `/dev/iio:deviceN`, `/sys/bus/iio/devices/iio:deviceN`, or a sysfs-like directory. |
| `expected_name` | empty | Exact IIO device name. Empty accepts `mpu6500` or `mpu6050`. |
| `verify_name` | `true` | Fail configuration when the device name is not accepted. `false` logs a warning instead. |
| `axis_map` | `x,y,z` | Body axis → sensor axis mapping. Must be a permutation, e.g. `y,-x,z`. |
| `gyro_scale` | `0` | rad/s per LSB. `0` reads `in_anglvel_<axis>_scale` (or `in_anglvel_scale`). |
| `accel_scale` | `0` | m/s² per LSB. `0` reads `in_accel_<axis>_scale` (or `in_accel_scale`). |
| `sampling_frequency_hz` | `0` | `0` leaves the device setting untouched. `>0` writes `sampling_frequency`. |
| `strict_sampling_frequency` | `false` | Fail configuration if `sampling_frequency` cannot be written. |
| `max_read_failures` | `3` | Consecutive read failures tolerated after a valid sample. `0` fails immediately. |
| `orientation.accel_gain` | `0.05` | Complementary filter proportional gain (1/s). |
| `orientation.integral_gain` | `0.0` | Gyro-bias integral gain (1/s²). `0` disables it. |
| `orientation.accel_tolerance` | `0.25` | Maximum relative deviation of \|a\| from 1 g for accelerometer correction. |

### Logging parameters

The package uses the process-global `common_cpp` logging backend. Supported
hardware parameters:

| Parameter | Default | Description |
|---|---|---|
| `log.log_level` | `info` | `trace`, `debug`, `info`, `warn`, `error`, `critical`, `off`. |
| `log.log_path` | `./log` | Directory for `log_<name>_<timestamp>.log`. |
| `log.async` | `true` | Asynchronous spdlog queue. |
| `log.logger_name` | `mpu6500_ros2_control` | Logger name. |
| `log.node_name` | `mpu6500_ros2_control` | Prefix shown in log lines. |
| `log.filter.black` | empty | Comma-separated filter rules (`start-`, `end-`, `inc-`, `equal-`). |

> The logging backend is initialized once per process. When two IMU components
> run in the same process, their `log.*` parameters must be identical; the
> defaults already are. Different `log.logger_name`/`log.node_name` values
> between components make the second initialization fail.

## Exported state interfaces

| Interface | Unit | Source |
|---|---|---|
| `orientation.x/y/z/w` | — | Complementary filter, body → world quaternion. |
| `angular_velocity.x/y/z` | rad/s | `in_anglvel_*_raw` × scale, body frame. |
| `linear_acceleration.x/y/z` | m/s² | `in_accel_*_raw` × scale, body frame. |

Coordinate convention follows ROS REP-103: x forward, y left, z up. An upright,
stationary sensor reads `(0, 0, +g)` and exports the identity quaternion.
`orientation` is produced from gravity and gyro only; yaw drifts and must not be
used as a long-term heading reference.

## Axis mounting

`axis_map` maps **body axes** to **sensor axes** and must be a permutation of
x/y/z; prefix a token with `-` to invert it. Examples:

| Mounting | `axis_map` | Body x / y / z read from |
|---|---|---|
| Default | `x,y,z` | sensor x, y, z |
| Rotated 90° about z | `y,-x,z` | sensor y, −sensor x, sensor z |
| Upside down | `x,-y,-z` | sensor x, −sensor y, −sensor z |

The same mapping is applied to gyro and acceleration.

## Lifecycle and failures

- `on_init`: parses parameters, validates `<sensor>` and state interfaces. No
  device access.
- `on_configure`: resolves the IIO directory, verifies `name`, opens the six raw
  attributes, caches `scale`, optionally writes `sampling_frequency`.
- `read`: `pread()` on the cached file descriptors; converts units, applies the
  axis map, updates the state interfaces and the filter.
- `on_cleanup` / `on_shutdown`: closes all file descriptors.
- First read failure (no previous sample): `ERROR`.
- Transient failures: keeps the last sample and returns `OK` for up to
  `max_read_failures` consecutive failures; the next failure returns `ERROR`.
  A successful read resets the counter.

No command interfaces are exported; this is a read-only sensor component.

## IIO requirements and current board status

Kernel support needed:

```text
CONFIG_INV_MPU6050_IIO=y
CONFIG_INV_MPU6050_SPI=y
```

Required sysfs attributes per device:

```text
/sys/bus/iio/devices/iio:deviceN/name
/sys/bus/iio/devices/iio:deviceN/in_anglvel_{x,y,z}_raw
/sys/bus/iio/devices/iio:deviceN/in_accel_{x,y,z}_raw
/sys/bus/iio/devices/iio:deviceN/in_anglvel_scale   (or per-channel)
/sys/bus/iio/devices/iio:deviceN/in_accel_scale     (or per-channel)
```

As of 2026-11-03 the `lubancat` board has the driver and device-tree node but
the SPI probe fails:

```text
inv-mpu6000-spi spi3.0: invalid whoami 0xff expected 0x70 (MPU6500)
```

No `iio:deviceN` for the IMU exists yet. Fix power, SPI3 wiring/CS, and
`spi-max-frequency` (try 1 MHz first) before using this package on hardware.
Until then, the unit tests exercise the same code path through a fake IIO sysfs
directory (`test/fake_mpu6500.hpp`).

## Testing with the fake MPU6500

The tests create a temporary directory that looks like IIO sysfs:

```text
fake_mpu6500_<pid>_<n>/
├─ name
├─ in_anglvel_scale
├─ in_accel_scale
├─ in_anglvel_x_raw ... in_anglvel_z_raw
└─ in_accel_x_raw   ... in_accel_z_raw
```

`FakeMpu6500::set_raw_gyro()` / `set_raw_accel()` rewrite the files, so the
driver's already-open descriptors read the new values. Run:

```bash
ctest --test-dir build-mpu6500 --output-on-failure
# or
colcon test --packages-select mpu6500_ros2_control --event-handlers console_direct+
```

Tests cover:

- scale conversion, axis map/sign, device-name checks, missing attributes,
  corrupt values, `sampling_frequency` write/failure, close/reopen;
- filter initialization, roll convergence, pure gyro integration, invalid `dt`;
- `SensorInterface` lifecycle, interface export, transient failure handling,
  cleanup/reconfigure, and two independent components (head + body);
- pluginlib loading from the build tree.

## Limitations

- Reads six sysfs attributes per cycle instead of using an IIO buffer. At 50 Hz
  this is negligible; for high-rate sampling use an IIO buffer/libiio backend.
- The `scale` attributes are cached at configure time; changing the device range
  at runtime requires a reconfigure.
- `sampling_frequency` may not be writable on all kernels; by default a failure
  is a warning, not an error.
- IIO device numbering is not stable across boots/probe order; configure by path
  after checking `name`. udev rules are recommended for production.
- No magnetometer: yaw is not absolute and drifts.
- One IIO device per plugin instance. Dual IMU is two components; do not average
  the two sensors — they are different rigid bodies.
