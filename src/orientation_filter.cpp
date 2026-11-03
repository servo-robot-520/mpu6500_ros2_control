#include "mpu6500_ros2_control/orientation_filter.hpp"

#include <algorithm>
#include <cmath>

namespace mpu6500_ros2_control {
    namespace {

        constexpr double kGravity = 9.80665;
        /// @brief 内部四元数：w, x, y, z。
        using Quaternion = std::array<double, 4>;

        /// @brief 归一化四元数；数值异常时退化为单位四元数。
        void normalize(Quaternion& q) {
            const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (!std::isfinite(norm) || norm < 1e-12) {
                q = Quaternion{1.0, 0.0, 0.0, 0.0};
                return;
            }
            for (double& value : q) {
                value /= norm;
            }
        }

        /// @brief 由 roll/pitch 构造 ZYX 顺序的四元数（yaw 取 0）。
        Quaternion from_roll_pitch(const double roll, const double pitch) {
            const double cr = std::cos(roll * 0.5);
            const double sr = std::sin(roll * 0.5);
            const double cp = std::cos(pitch * 0.5);
            const double sp = std::sin(pitch * 0.5);
            return Quaternion{cp * cr, cp * sr, sp * cr, -sp * sr};
        }

        /// @brief 计算世界坐标 +z 轴在机体系中的方向，即姿态下的“上”方向。
        ///
        /// body->world 旋转矩阵 R 的第三行为 R^T * (0,0,1)，
        /// 即静止时加速度计应测得的方向。
        std::array<double, 3> estimated_up(const Quaternion& q) {
            const double w = q[0];
            const double x = q[1];
            const double y = q[2];
            const double z = q[3];
            return {
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            };
        }

        /// @brief 三维叉积。
        std::array<double, 3> cross(const std::array<double, 3>& lhs,
                                    const std::array<double, 3>& rhs) {
            return {
                lhs[1] * rhs[2] - lhs[2] * rhs[1],
                lhs[2] * rhs[0] - lhs[0] * rhs[2],
                lhs[0] * rhs[1] - lhs[1] * rhs[0],
            };
        }

    } // namespace

    OrientationFilter::OrientationFilter(const OrientationFilterOptions& options) :
        options_(options) {
    }

    void OrientationFilter::reset() {
        q_ = Quaternion{1.0, 0.0, 0.0, 0.0};
        integral_ = {0.0, 0.0, 0.0};
        initialized_ = false;
    }

    void OrientationFilter::update(const std::array<double, 3>& angular_velocity,
                                   const std::array<double, 3>& linear_acceleration,
                                   const double dt) {
        const double ax = linear_acceleration[0];
        const double ay = linear_acceleration[1];
        const double az = linear_acceleration[2];
        const double accel_norm = std::sqrt(ax * ax + ay * ay + az * az);

        // 第一帧只用重力确定 roll/pitch，yaw 置 0；没有有效加速度时继续保持未初始化。
        if (!initialized_) {
            if (!std::isfinite(accel_norm) || accel_norm < 1e-6) {
                return;
            }
            const double roll = std::atan2(ay, az);
            const double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
            q_ = from_roll_pitch(roll, pitch);
            normalize(q_);
            initialized_ = true;
            return;
        }

        if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
            return;
        }

        std::array<double, 3> omega = angular_velocity;

        // 只有在加速度模长接近 1 g 时才认为它主要反映重力，避免运动加速度污染姿态。
        const bool accel_valid = std::isfinite(accel_norm) && accel_norm > 1e-6 &&
            std::abs(accel_norm - kGravity) <= kGravity * std::max(0.0, options_.accel_tolerance);
        if (accel_valid) {
            const std::array<double, 3> measured{ax / accel_norm, ay / accel_norm, az / accel_norm};
            const std::array<double, 3> up = estimated_up(q_);
            // 世界固定向量在机体系中的变化率为 v_dot = -omega × v，
            // 因此把“估计上方向”转向“实测重力方向”的角速度修正为 measured × up。
            const std::array<double, 3> error = cross(measured, up);
            for (int axis = 0; axis < 3; ++axis) {
                integral_[axis] += options_.integral_gain * error[axis] * dt;
                omega[axis] += options_.accel_gain * error[axis] + integral_[axis];
            }
        }

        // 四元数微分：q_dot = 0.5 * q ⊗ omega，omega 为机体系角速度。
        const double half_dt = 0.5 * dt;
        const double w = q_[0];
        const double x = q_[1];
        const double y = q_[2];
        const double z = q_[3];
        q_[0] += half_dt * (-x * omega[0] - y * omega[1] - z * omega[2]);
        q_[1] += half_dt * (w * omega[0] + y * omega[2] - z * omega[1]);
        q_[2] += half_dt * (w * omega[1] + z * omega[0] - x * omega[2]);
        q_[3] += half_dt * (w * omega[2] + x * omega[1] - y * omega[0]);
        normalize(q_);
    }

    std::array<double, 4> OrientationFilter::orientation() const {
        return {q_[1], q_[2], q_[3], q_[0]};
    }

} // namespace mpu6500_ros2_control
