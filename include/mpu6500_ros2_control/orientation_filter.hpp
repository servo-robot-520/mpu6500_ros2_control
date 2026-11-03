#pragma once

#include <array>

namespace mpu6500_ros2_control {

    /// @brief 互补滤波器的可调参数。
    struct OrientationFilterOptions {
        /// 加速度计比例校正增益，单位 1/s，越大收敛越快但越易受线性加速度影响。
        double accel_gain{0.05};
        /// 加速度计积分校正增益，单位 1/s²，用于估计陀螺零偏；0 表示关闭。
        double integral_gain{0.0};
        /// 加速度模长相对 g 的最大允许偏差比例，超出该范围时本帧只用陀螺。
        double accel_tolerance{0.25};
    };

    /// @brief 基于加速度计与陀螺的互补姿态滤波器。
    ///
    /// MPU6500 没有磁力计，也没有板载四元数。本滤波器输出 body->world 四元数：
    /// - roll/pitch 由重力方向长期约束；
    /// - yaw 仅由陀螺积分得到，会缓慢漂移；
    /// - 当加速度模长明显偏离 1 g（剧烈运动）时，本帧不做加速度校正。
    ///
    /// 坐标系约定与 ROS REP-103 一致：静止直立时 body z 轴朝上，
    /// 加速度计读数为 (0, 0, +g)，四元数为 (x=0, y=0, z=0, w=1)。
    class OrientationFilter {
    public:
        explicit OrientationFilter(const OrientationFilterOptions& options = OrientationFilterOptions{});

        /// @brief 清空姿态与积分项，下一次 update() 重新用重力初始化。
        void reset();

        /// @param angular_velocity 机体角速度，rad/s。
        /// @param linear_acceleration 机体系比力，m/s²。
        /// @param dt 采样间隔，s；非法值会被忽略。
        void update(const std::array<double, 3>& angular_velocity,
                    const std::array<double, 3>& linear_acceleration,
                    double dt);

        /// @brief 当前姿态四元数，顺序为 x, y, z, w。
        [[nodiscard]] std::array<double, 4> orientation() const;

        /// @brief 是否已经用一帧有效加速度完成初始化。
        [[nodiscard]] bool initialized() const {
            return initialized_;
        }

    private:
        OrientationFilterOptions options_;
        /// 内部四元数顺序为 w, x, y, z，便于直接使用四元数乘法公式。
        std::array<double, 4> q_{1.0, 0.0, 0.0, 0.0};
        /// 加速度计积分校正累计量，用于补偿陀螺零偏。
        std::array<double, 3> integral_{};
        bool initialized_{false};
    };

} // namespace mpu6500_ros2_control
