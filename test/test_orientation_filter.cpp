#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "mpu6500_ros2_control/orientation_filter.hpp"

namespace mpu6500_ros2_control {
    namespace {

        constexpr double kGravity = 9.80665;
        constexpr double kPi = 3.14159265358979323846;
        const std::array<double, 4> kIdentityQuaternion{0.0, 0.0, 0.0, 1.0};

        double quaternion_norm(const std::array<double, 4>& q) {
            return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        }

        TEST(OrientationFilter, InitialisesFromGravityAndStaysUpright) {
            OrientationFilter filter;
            EXPECT_FALSE(filter.initialized());

            filter.update({0.0, 0.0, 0.0}, {0.0, 0.0, kGravity}, 0.01);
            EXPECT_TRUE(filter.initialized());

            for (int index = 0; index < 100; ++index) {
                filter.update({0.0, 0.0, 0.0}, {0.0, 0.0, kGravity}, 0.01);
            }

            const auto q = filter.orientation();
            EXPECT_NEAR(q[0], 0.0, 1e-9);
            EXPECT_NEAR(q[1], 0.0, 1e-9);
            EXPECT_NEAR(q[2], 0.0, 1e-9);
            EXPECT_NEAR(q[3], 1.0, 1e-9);
            EXPECT_NEAR(quaternion_norm(q), 1.0, 1e-12);
        }

        TEST(OrientationFilter, ConvergesToMeasuredRoll) {
            OrientationFilterOptions options;
            options.accel_gain = 1.0;
            OrientationFilter filter(options);

            filter.update({0.0, 0.0, 0.0}, {0.0, 0.0, kGravity}, 0.01);
            // 加速度计只读到 +y 方向的重力，对应 roll ≈ +90°。
            for (int index = 0; index < 500; ++index) {
                filter.update({0.0, 0.0, 0.0}, {0.0, kGravity, 0.0}, 0.01);
            }

            const auto q = filter.orientation();
            EXPECT_GT(q[0], 0.69);
            EXPECT_NEAR(q[1], 0.0, 1e-6);
            EXPECT_NEAR(q[2], 0.0, 1e-6);
            EXPECT_GT(q[3], 0.69);
            EXPECT_NEAR(quaternion_norm(q), 1.0, 1e-12);
        }

        TEST(OrientationFilter, IntegratesGyroWhenAccelerationIsUnavailable) {
            OrientationFilter filter;
            filter.update({0.0, 0.0, 0.0}, {0.0, 0.0, kGravity}, 0.01);

            // 100 步 × 0.01 s × π/2 rad/s = 90° yaw。
            for (int index = 0; index < 100; ++index) {
                filter.update({0.0, 0.0, kPi / 2.0}, {0.0, 0.0, 0.0}, 0.01);
            }

            const auto q = filter.orientation();
            EXPECT_NEAR(q[0], 0.0, 1e-6);
            EXPECT_NEAR(q[1], 0.0, 1e-6);
            EXPECT_NEAR(q[2], std::sin(kPi / 4.0), 1e-3);
            EXPECT_NEAR(q[3], std::cos(kPi / 4.0), 1e-3);
        }

        TEST(OrientationFilter, IgnoresInvalidPeriod) {
            OrientationFilter filter;
            filter.update({0.0, 0.0, 0.0}, {0.0, 0.0, kGravity}, 0.01);
            const auto before = filter.orientation();

            filter.update({10.0, 10.0, 10.0}, {0.0, 0.0, kGravity}, 0.0);
            filter.update({10.0, 10.0, 10.0}, {0.0, 0.0, kGravity}, 2.0);
            filter.update({10.0, 10.0, 10.0}, {0.0, 0.0, kGravity}, -0.01);

            EXPECT_EQ(filter.orientation(), before);
        }

        TEST(OrientationFilter, DoesNotInitialiseWithoutValidAcceleration) {
            OrientationFilter filter;
            filter.update({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, 0.01);
            EXPECT_FALSE(filter.initialized());
            EXPECT_EQ(filter.orientation(), kIdentityQuaternion);
        }

        TEST(OrientationFilter, ResetRestoresUninitialisedState) {
            OrientationFilter filter;
            filter.update({0.0, 0.0, 0.0}, {0.0, 0.0, kGravity}, 0.01);
            ASSERT_TRUE(filter.initialized());

            filter.reset();
            EXPECT_FALSE(filter.initialized());
            EXPECT_EQ(filter.orientation(), kIdentityQuaternion);
        }

    } // namespace
} // namespace mpu6500_ros2_control
