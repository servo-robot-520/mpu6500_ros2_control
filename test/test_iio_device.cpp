#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "fake_mpu6500.hpp"
#include "mpu6500_ros2_control/mpu6500_iio_device.hpp"

namespace mpu6500_ros2_control {
    namespace {

        using test::FakeMpu6500;

        IioDeviceOptions default_options(const FakeMpu6500& fake) {
            IioDeviceOptions options;
            options.device_path = fake.path();
            return options;
        }

        class IioDeviceTest : public ::testing::Test {
        protected:
            void SetUp() override {
                fake_ = std::make_unique<FakeMpu6500>();
            }

            std::unique_ptr<FakeMpu6500> fake_;
        };

        TEST_F(IioDeviceTest, OpensAndReadsScaledSample) {
            fake_->set_raw_gyro(100, -200, 300);
            fake_->set_raw_accel(10, 20, -30);

            Mpu6500IioDevice device;
            IioDeviceInfo info;
            const auto result = device.open(default_options(*fake_), &info);
            ASSERT_TRUE(result) << result.error();

            EXPECT_EQ(info.device_name, "mpu6500");
            EXPECT_NEAR(info.gyro_scale[0], FakeMpu6500::kGyroScale, 1e-15);
            EXPECT_NEAR(info.accel_scale[2], FakeMpu6500::kAccelScale, 1e-15);

            ImuSample sample;
            ASSERT_TRUE(device.read(&sample));
            EXPECT_NEAR(sample.angular_velocity[0], 100.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(sample.angular_velocity[1], -200.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(sample.angular_velocity[2], 300.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(sample.linear_acceleration[0], 10.0 * FakeMpu6500::kAccelScale, 1e-12);
            EXPECT_NEAR(sample.linear_acceleration[1], 20.0 * FakeMpu6500::kAccelScale, 1e-12);
            EXPECT_NEAR(sample.linear_acceleration[2], -30.0 * FakeMpu6500::kAccelScale, 1e-12);
        }

        TEST_F(IioDeviceTest, AppliesAxisMapAndSign) {
            fake_->set_raw_gyro(100, 200, 300);
            fake_->set_raw_accel(10, 20, 30);

            auto options = default_options(*fake_);
            options.axis_map = {1, 0, 2};
            options.axis_sign = {1.0, -1.0, 1.0};

            Mpu6500IioDevice device;
            ASSERT_TRUE(device.open(options));

            ImuSample sample;
            ASSERT_TRUE(device.read(&sample));
            EXPECT_NEAR(sample.angular_velocity[0], 200.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(sample.angular_velocity[1], -100.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(sample.angular_velocity[2], 300.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(sample.linear_acceleration[0], 20.0 * FakeMpu6500::kAccelScale, 1e-12);
            EXPECT_NEAR(sample.linear_acceleration[1], -10.0 * FakeMpu6500::kAccelScale, 1e-12);
            EXPECT_NEAR(sample.linear_acceleration[2], 30.0 * FakeMpu6500::kAccelScale, 1e-12);
        }

        TEST_F(IioDeviceTest, RejectsMismatchedNameUnlessVerificationIsDisabled) {
            fake_->set_name("bmp280");

            auto options = default_options(*fake_);
            Mpu6500IioDevice device;
            EXPECT_FALSE(device.open(options));
            EXPECT_FALSE(device.is_open());

            options.verify_name = false;
            IioDeviceInfo info;
            ASSERT_TRUE(device.open(options, &info));
            EXPECT_EQ(info.device_name, "bmp280");
            EXPECT_FALSE(info.warnings.empty());
        }

        TEST_F(IioDeviceTest, AcceptsMpu6050CompatibleDriverName) {
            fake_->set_name("mpu6050");

            Mpu6500IioDevice device;
            EXPECT_TRUE(device.open(default_options(*fake_)));
        }

        TEST_F(IioDeviceTest, RejectsDuplicateAxisMapping) {
            auto options = default_options(*fake_);
            options.axis_map = {0, 0, 2};

            Mpu6500IioDevice device;
            EXPECT_FALSE(device.open(options));
        }

        TEST_F(IioDeviceTest, UsesExplicitScaleWhenSysfsScaleIsMissing) {
            fake_->remove_file("in_anglvel_scale");
            fake_->remove_file("in_accel_scale");
            fake_->set_raw_gyro(100, 0, 0);
            fake_->set_raw_accel(0, 0, 1000);

            auto options = default_options(*fake_);
            options.gyro_scale = 0.001;
            options.accel_scale = 0.01;

            Mpu6500IioDevice device;
            ASSERT_TRUE(device.open(options));

            ImuSample sample;
            ASSERT_TRUE(device.read(&sample));
            EXPECT_NEAR(sample.angular_velocity[0], 0.1, 1e-12);
            EXPECT_NEAR(sample.linear_acceleration[2], 10.0, 1e-12);
        }

        TEST_F(IioDeviceTest, FailsWhenScaleIsMissingAndNotOverridden) {
            fake_->remove_file("in_anglvel_scale");

            Mpu6500IioDevice device;
            const auto result = device.open(default_options(*fake_));
            EXPECT_FALSE(result);
            EXPECT_NE(result.error().find("scale"), std::string::npos);
        }

        TEST_F(IioDeviceTest, FailsWhenRawChannelIsMissing) {
            fake_->remove_file("in_accel_y_raw");

            Mpu6500IioDevice device;
            EXPECT_FALSE(device.open(default_options(*fake_)));
        }

        TEST_F(IioDeviceTest, ReportsCorruptRawValue) {
            Mpu6500IioDevice device;
            ASSERT_TRUE(device.open(default_options(*fake_)));

            ImuSample sample;
            ASSERT_TRUE(device.read(&sample));

            fake_->write_file("in_anglvel_x_raw", "");
            const auto result = device.read(&sample);
            EXPECT_FALSE(result);
            EXPECT_NE(result.error().find("in_anglvel_x_raw"), std::string::npos);
        }

        TEST_F(IioDeviceTest, WritesSamplingFrequencyWhenAvailable) {
            fake_->write_file("sampling_frequency", "200");

            auto options = default_options(*fake_);
            options.sampling_frequency_hz = 100.0;

            Mpu6500IioDevice device;
            ASSERT_TRUE(device.open(options));
            EXPECT_NE(fake_->read_file("sampling_frequency").find("100"), std::string::npos);
        }

        TEST_F(IioDeviceTest, WarnsOrFailsWhenSamplingFrequencyCannotBeWritten) {
            fake_->remove_file("sampling_frequency");

            auto options = default_options(*fake_);
            options.sampling_frequency_hz = 100.0;

            Mpu6500IioDevice device;
            IioDeviceInfo info;
            ASSERT_TRUE(device.open(options, &info));
            EXPECT_FALSE(info.warnings.empty());
            device.close();

            options.strict_sampling_frequency = true;
            EXPECT_FALSE(device.open(options));
        }

        TEST_F(IioDeviceTest, RejectsMissingDeviceDirectory) {
            IioDeviceOptions options;
            options.device_path = "/dev/iio:device_no_such_mpu6500";

            Mpu6500IioDevice device;
            EXPECT_FALSE(device.open(options));
        }

        TEST_F(IioDeviceTest, ClosesAndReopens) {
            Mpu6500IioDevice device;
            ASSERT_TRUE(device.open(default_options(*fake_)));
            EXPECT_TRUE(device.is_open());

            device.close();
            EXPECT_FALSE(device.is_open());

            ASSERT_TRUE(device.open(default_options(*fake_)));
            EXPECT_TRUE(device.is_open());
        }

    } // namespace
} // namespace mpu6500_ros2_control
