#pragma once

#include <array>
#include <string>
#include <vector>

#include "mpu6500_ros2_control/result.hpp"

namespace mpu6500_ros2_control {

    /// @brief 一次 MPU6500 采样，已换算为 SI 单位并完成轴系映射。
    struct ImuSample {
        /// 角速度，rad/s，车体系。
        std::array<double, 3> angular_velocity{};
        /// 加速度，m/s²，车体系。
        std::array<double, 3> linear_acceleration{};
    };

    /// @brief IIO 设备打开参数。
    struct IioDeviceOptions {
        /// IIO 设备路径，支持：
        /// - /dev/iio:deviceN
        /// - /sys/bus/iio/devices/iio:deviceN（或 /sys/class/iio 下的符号链接）
        /// - 任意包含同名属性的目录（测试用的模拟 sysfs）。
        std::string device_path;
        /// 期望的设备名；为空时接受 mpu6500 / mpu6050。
        std::string expected_name;
        /// 是否校验 name 属性；关闭后未知设备只给出告警。
        bool verify_name{true};
        /// 车体轴 -> 传感器轴映射。例如 {1, 0, 2} 表示车体 x 取传感器 y。
        std::array<int, 3> axis_map{0, 1, 2};
        /// 车体轴符号，与 axis_map 一一对应。
        std::array<double, 3> axis_sign{1.0, 1.0, 1.0};
        /// 角速度/加速度 scale；小于等于 0 时从 sysfs 的 scale 文件读取。
        double gyro_scale{0.0};
        double accel_scale{0.0};
        /// 大于 0 时尝试写入 sampling_frequency（Hz）。
        double sampling_frequency_hz{0.0};
        /// sampling_frequency 写入失败时是否判定为错误。
        bool strict_sampling_frequency{false};
    };

    /// @brief 打开设备后得到的实际配置，便于插件打印诊断信息。
    struct IioDeviceInfo {
        std::string device_directory;
        std::string device_name;
        std::array<double, 3> gyro_scale{};
        std::array<double, 3> accel_scale{};
        std::vector<std::string> warnings;
    };

    /// @brief 通过 Linux IIO sysfs 读取 MPU6500 原始样本的驱动。
    ///
    /// 设备在 open() 时一次性打开所有 raw 属性文件并缓存 scale，read() 只做
    /// pread()，不会每个周期重新打开文件。该类不依赖 ROS，可在普通 C++ 或
    /// CMake 工程中单独使用。
    class Mpu6500IioDevice {
    public:
        Mpu6500IioDevice() = default;
        ~Mpu6500IioDevice();

        Mpu6500IioDevice(const Mpu6500IioDevice&) = delete;
        Mpu6500IioDevice& operator=(const Mpu6500IioDevice&) = delete;

        /// @brief 解析设备路径、校验设备名、打开通道并读取 scale。
        /// @param options 设备与轴系配置。
        /// @param info 可选输出，返回实际设备目录、名称、scale 与告警。
        /// @return 成功或携带错误信息。
        Result open(const IioDeviceOptions& options, IioDeviceInfo* info = nullptr);

        /// @brief 关闭所有已打开的属性文件；可重复调用。
        void close();

        /// @brief 设备是否处于已打开状态。
        [[nodiscard]] bool is_open() const;

        /// @brief 读取一帧原始数据并换算为 SI 单位。
        /// @param sample 输出样本，仅在成功时有效。
        /// @return 成功或携带错误信息。
        Result read(ImuSample* sample) const;

    private:
        static constexpr int kChannelCount = 6;

        std::array<int, kChannelCount> fds_{-1, -1, -1, -1, -1, -1};
        std::string device_directory_;
        std::string device_name_;
        std::array<double, 3> gyro_scale_{};
        std::array<double, 3> accel_scale_{};
        std::array<int, 3> axis_map_{0, 1, 2};
        std::array<double, 3> axis_sign_{1.0, 1.0, 1.0};
    };

} // namespace mpu6500_ros2_control
