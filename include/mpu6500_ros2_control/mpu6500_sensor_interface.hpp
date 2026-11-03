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
    /// - orientation_valid（可选）：姿态是否已收敛且传感器健康，1=可用，0=不可用。
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
            /// 姿态有效标志：1.0=可用，0.0=不可用。
            ///
            /// 它不是“本帧数据新鲜度”，而是“滤波器已收敛且传感器健康”：
            /// - 启动后必须连续累计 valid_samples 个 |a|≈g 的样本才置 1，
            ///   避免把上电瞬间的运动加速度当成重力；
            /// - 置 1 后，运动中的线性加速度只会跳过本帧姿态校正，不会清零；
            /// - 短暂读取失败（<= max_read_failures）不清零；持续失败、cleanup
            ///   或 reset 时清零；
            /// - yaw 不在有效性保证范围内：MPU6500 无磁力计，yaw 始终会漂移。
            ///
            /// 控制器应 claim 本接口，在它为 0 时保持 hold/home、暂不启动策略，
            /// 跌倒检测也不投票；等为 1 后再进入正常控制。
            double orientation_valid{0.0};
        };

        /// @brief 插件级配置。
        struct Options {
            IioDeviceOptions device;
            OrientationFilterOptions filter;
            /// 连续读取失败多少次后向 ResourceManager 报告 ERROR。
            int max_read_failures{3};
            /// 启动阶段需要连续多少个 |a|≈g 的样本才把 orientation_valid 置 1。
            /// 默认 25；50 Hz 下约 0.5 s，避免单帧偶然值被当成可信姿态。
            int valid_samples{25};
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
        /// 启动阶段连续有效加速度样本计数；orientation_valid 置 1 后不再使用。
        int consecutive_valid_samples_{0};
        bool has_sample_{false};
        int64_t last_read_time_ns_{0};
    };

} // namespace mpu6500_ros2_control
