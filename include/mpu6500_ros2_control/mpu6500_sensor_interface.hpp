#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "mpu6500_ros2_control/mpu6500_iio_device.hpp"
#include "mpu6500_ros2_control/orientation_filter.hpp"

namespace mpu6500_ros2_control {

    /// @brief MPU6500 的 ros2_control SensorInterface 插件。
    ///
    /// 一个实例对应一个 IIO 设备（一个 MPU6500），不包含“头部/躯干”语义。
    /// 双 IMU 时在 URDF 中声明两个 type="sensor" 的 ros2_control 组件，例如
    /// head_imu（默认 CS0）与 body_imu（默认 CS1）；是否启用由 URDF 决定。
    ///
    /// 导出的状态接口：
    /// - orientation.x/y/z/w
    /// - angular_velocity.x/y/z
    /// - linear_acceleration.x/y/z
    class MPU6500SensorInterface : public hardware_interface::SensorInterface {
    public:
        /// @brief 解析硬件参数并校验 URDF 中声明的状态接口，不访问设备。
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        /// @brief 打开 IIO 设备、读取 scale 并准备姿态滤波器。
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        /// @brief 关闭 IIO 设备并释放文件描述符。
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state) override;

        /// @brief 进程关闭前释放设备。
        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state) override;

        /// @brief 传感器没有输出，激活时无需额外动作。
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        /// @brief 传感器没有输出，停用时无需额外动作。
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        /// @brief 导出 URDF 中声明的状态接口。
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /// @brief 读取一帧 IMU 数据并更新状态接口与姿态四元数。
        hardware_interface::return_type read(const rclcpp::Time& time,
                                             const rclcpp::Duration& period) override;

    private:
        /// @brief 导出的状态数据；所有 StateInterface 指向这里的字段。
        struct State {
            /// 姿态四元数，顺序为 x, y, z, w。
            std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
            /// 角速度，rad/s。
            std::array<double, 3> angular_velocity{};
            /// 加速度，m/s²。
            std::array<double, 3> linear_acceleration{};
        };

        /// @brief 插件级配置。
        struct Options {
            IioDeviceOptions device;
            OrientationFilterOptions filter;
            /// 连续读取失败多少次后向 ResourceManager 报告 ERROR。
            int max_read_failures{3};
        };

        /// @brief 解析并校验 URDF 硬件参数。
        hardware_interface::CallbackReturn parse_options(const char* caller);

        /// @brief 清空运行时状态与滤波器。
        void reset_runtime_state();

        /// @brief 计算滤波积分使用的采样周期。
        double sample_period(const rclcpp::Time& time, const rclcpp::Duration& period);

        /// @brief 把接口名映射到 State 中的字段；未知接口返回 nullptr。
        [[nodiscard]] double* interface_pointer(const std::string& name);

        Options options_;
        State state_;
        Mpu6500IioDevice device_;
        OrientationFilter filter_;
        int consecutive_failures_{0};
        bool has_sample_{false};
        int64_t last_read_time_ns_{0};
    };

} // namespace mpu6500_ros2_control
