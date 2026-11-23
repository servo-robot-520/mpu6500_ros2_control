#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "fake_mpu6500.hpp"
#include "mpu6500_ros2_control/mpu6500_sensor_interface.hpp"

namespace mpu6500_ros2_control {
    namespace {

        using test::FakeMpu6500;

        constexpr double kPi = 3.14159265358979323846;

        hardware_interface::ComponentInfo make_sensor(const std::string& name,
                                                      const bool include_valid = false) {
            hardware_interface::ComponentInfo sensor;
            sensor.name = name;
            for (const char* interface_name :
                 {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
                  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
                  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}) {
                hardware_interface::InterfaceInfo interface;
                interface.name = interface_name;
                sensor.state_interfaces.push_back(interface);
            }
            if (include_valid) {
                hardware_interface::InterfaceInfo interface;
                interface.name = "orientation_valid";
                sensor.state_interfaces.push_back(interface);
            }
            return sensor;
        }

        hardware_interface::HardwareInfo make_hardware_info(const std::string& device_path,
                                                            const std::string& sensor_name = "imu0",
                                                            const bool include_valid = false) {
            hardware_interface::HardwareInfo info;
            info.name = sensor_name;
            info.type = "sensor";
            info.hardware_class_type = "mpu6500_ros2_control/MPU6500SensorInterface";
            info.hardware_parameters["iio_device"] = device_path;
            info.sensors.push_back(make_sensor(sensor_name, include_valid));
            return info;
        }

        bool init_and_configure(MPU6500SensorInterface& hardware,
                                const hardware_interface::HardwareInfo& info) {
            if (hardware.on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
                return false;
            }
            return hardware.on_configure(rclcpp_lifecycle::State()) ==
                hardware_interface::CallbackReturn::SUCCESS;
        }

        class SensorInterfaceTest : public ::testing::Test {
        protected:
            void SetUp() override {
                fake_ = std::make_unique<FakeMpu6500>();
            }

            std::unique_ptr<FakeMpu6500> fake_;
        };

        TEST_F(SensorInterfaceTest, InitialisesAndExportsDeclaredInterfaces) {
            auto info = make_hardware_info(fake_->path());

            MPU6500SensorInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

            const auto states = hardware.export_state_interfaces();
            ASSERT_EQ(states.size(), 10U);
            EXPECT_EQ(states[0].get_name(), "imu0/orientation.x");
            EXPECT_EQ(states[3].get_name(), "imu0/orientation.w");
            EXPECT_EQ(states[4].get_name(), "imu0/angular_velocity.x");
            EXPECT_EQ(states[9].get_name(), "imu0/linear_acceleration.z");
        }

        TEST_F(SensorInterfaceTest, InitDoesNotAccessDevice) {
            auto info = make_hardware_info("/dev/iio:device_no_such_mpu6500");

            MPU6500SensorInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State()),
                      hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(SensorInterfaceTest, RejectsMissingDeviceParameter) {
            auto info = make_hardware_info(fake_->path());
            info.hardware_parameters.erase("iio_device");

            MPU6500SensorInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(SensorInterfaceTest, RejectsUnsupportedStateInterface) {
            auto info = make_hardware_info(fake_->path());
            info.sensors[0].state_interfaces[0].name = "temperature";

            MPU6500SensorInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(SensorInterfaceTest, RejectsEmptySensorDeclaration) {
            auto info = make_hardware_info(fake_->path());
            info.sensors.clear();

            MPU6500SensorInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(SensorInterfaceTest, RejectsMultipleSensorsPerComponent) {
            auto info = make_hardware_info(fake_->path());
            info.sensors.push_back(make_sensor("imu1"));

            MPU6500SensorInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(SensorInterfaceTest, RejectsInvalidAxisMap) {
            auto info = make_hardware_info(fake_->path());
            info.hardware_parameters["axis_map"] = "x,x,z";

            MPU6500SensorInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(SensorInterfaceTest, RejectsInvalidMaxReadFailures) {
            auto info = make_hardware_info(fake_->path());
            info.hardware_parameters["max_read_failures"] = "-1";

            MPU6500SensorInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

        TEST_F(SensorInterfaceTest, ReadsAndConvertsSample) {
            auto info = make_hardware_info(fake_->path());
            fake_->set_raw_gyro(100, 200, -300);
            fake_->set_raw_accel(0, 0, FakeMpu6500::kGravityRaw);

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));

            auto states = hardware.export_state_interfaces();
            ASSERT_EQ(states.size(), 10U);
            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);

            EXPECT_NEAR(states[0].get_value(), 0.0, 1e-9);
            EXPECT_NEAR(states[1].get_value(), 0.0, 1e-9);
            EXPECT_NEAR(states[2].get_value(), 0.0, 1e-9);
            EXPECT_NEAR(states[3].get_value(), 1.0, 1e-9);
            EXPECT_NEAR(states[4].get_value(), 100.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(states[5].get_value(), 200.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(states[6].get_value(), -300.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(states[7].get_value(), 0.0, 1e-12);
            EXPECT_NEAR(states[8].get_value(), 0.0, 1e-12);
            EXPECT_NEAR(states[9].get_value(),
                        static_cast<double>(FakeMpu6500::kGravityRaw) * FakeMpu6500::kAccelScale, 1e-9);
        }

        TEST_F(SensorInterfaceTest, AppliesAxisMapParameter) {
            auto info = make_hardware_info(fake_->path());
            info.hardware_parameters["axis_map"] = "y,-x,z";
            fake_->set_raw_gyro(100, 200, 300);
            fake_->set_raw_accel(10, 20, 30);

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            auto states = hardware.export_state_interfaces();

            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_NEAR(states[4].get_value(), 200.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(states[5].get_value(), -100.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(states[6].get_value(), 300.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(states[7].get_value(), 20.0 * FakeMpu6500::kAccelScale, 1e-12);
            EXPECT_NEAR(states[8].get_value(), -10.0 * FakeMpu6500::kAccelScale, 1e-12);
            EXPECT_NEAR(states[9].get_value(), 30.0 * FakeMpu6500::kAccelScale, 1e-12);
        }

        TEST_F(SensorInterfaceTest, OrientationFollowsRollingAcceleration) {
            auto info = make_hardware_info(fake_->path());
            info.hardware_parameters["orientation.accel_gain"] = "1.0";

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            auto states = hardware.export_state_interfaces();

            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            fake_->set_raw_accel(0, FakeMpu6500::kGravityRaw, 0);
            for (int index = 0; index < 500; ++index) {
                ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                          hardware_interface::return_type::OK);
            }

            EXPECT_GT(states[0].get_value(), 0.69);
            EXPECT_GT(states[3].get_value(), 0.69);
        }

        TEST_F(SensorInterfaceTest, KeepsLastSampleOnTransientReadFailure) {
            auto info = make_hardware_info(fake_->path());
            fake_->set_raw_gyro(100, 0, 0);

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            auto states = hardware.export_state_interfaces();

            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            const double gyro_x = states[4].get_value();

            fake_->write_file("in_anglvel_x_raw", "");
            for (int attempt = 1; attempt <= 3; ++attempt) {
                EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                          hardware_interface::return_type::OK);
                EXPECT_DOUBLE_EQ(states[4].get_value(), gyro_x);
            }
            EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::ERROR);

            fake_->set_raw_gyro(200, 0, 0);
            EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_NEAR(states[4].get_value(), 200.0 * FakeMpu6500::kGyroScale, 1e-12);
        }

        TEST_F(SensorInterfaceTest, FailsImmediatelyBeforeFirstValidSample) {
            auto info = make_hardware_info(fake_->path());

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));

            fake_->write_file("in_accel_x_raw", "");
            EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::ERROR);
        }

        TEST_F(SensorInterfaceTest, ReadBeforeConfigureFails) {
            auto info = make_hardware_info(fake_->path());

            MPU6500SensorInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::ERROR);
        }

        TEST_F(SensorInterfaceTest, CleansUpAndReconfigures) {
            auto info = make_hardware_info(fake_->path());

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            EXPECT_EQ(hardware.on_cleanup(rclcpp_lifecycle::State()),
                      hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State()),
                      hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
        }

        TEST_F(SensorInterfaceTest, ReadsTwoIndependentImuComponents) {
            FakeMpu6500 imu0_fake;
            FakeMpu6500 imu1_fake;

            auto imu0_info = make_hardware_info(imu0_fake.path(), "imu0");
            auto imu1_info = make_hardware_info(imu1_fake.path(), "imu1");

            MPU6500SensorInterface imu0;
            MPU6500SensorInterface imu1;
            ASSERT_TRUE(init_and_configure(imu0, imu0_info));
            ASSERT_TRUE(init_and_configure(imu1, imu1_info));

            auto imu0_states = imu0.export_state_interfaces();
            auto imu1_states = imu1.export_state_interfaces();
            ASSERT_EQ(imu0_states.size(), 10U);
            ASSERT_EQ(imu1_states.size(), 10U);
            EXPECT_EQ(imu0_states[0].get_name(), "imu0/orientation.x");
            EXPECT_EQ(imu1_states[0].get_name(), "imu1/orientation.x");

            imu0_fake.set_raw_gyro(100, 0, 0);
            imu1_fake.set_raw_gyro(-200, 0, 0);

            ASSERT_EQ(imu0.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            ASSERT_EQ(imu1.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);

            EXPECT_NEAR(imu0_states[4].get_value(), 100.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(imu1_states[4].get_value(), -200.0 * FakeMpu6500::kGyroScale, 1e-12);
            EXPECT_NEAR(imu0_states[4].get_value(), -0.5 * imu1_states[4].get_value(), 1e-12);
        }


        TEST_F(SensorInterfaceTest, ExportsOrientationValidWhenDeclared) {
            auto info = make_hardware_info(fake_->path(), "imu0", true);

            MPU6500SensorInterface hardware;
            ASSERT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::SUCCESS);

            const auto states = hardware.export_state_interfaces();
            ASSERT_EQ(states.size(), 11U);
            EXPECT_EQ(states[10].get_name(), "imu0/orientation_valid");
            EXPECT_DOUBLE_EQ(states[10].get_value(), 0.0);
        }

        TEST_F(SensorInterfaceTest, OrientationValidRequiresConsecutiveGravitySamples) {
            auto info = make_hardware_info(fake_->path(), "imu0", true);
            info.hardware_parameters["orientation.valid_samples"] = "5";

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            auto states = hardware.export_state_interfaces();
            ASSERT_EQ(states.size(), 11U);

            for (int sample = 1; sample < 5; ++sample) {
                ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                          hardware_interface::return_type::OK);
                EXPECT_DOUBLE_EQ(states[10].get_value(), 0.0) << "sample " << sample;
            }
            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 1.0);
        }

        TEST_F(SensorInterfaceTest, OrientationValidWaitsForGravityAtStartup) {
            auto info = make_hardware_info(fake_->path(), "imu0", true);
            info.hardware_parameters["orientation.valid_samples"] = "5";
            fake_->set_raw_accel(0, 0, 0);

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            auto states = hardware.export_state_interfaces();

            // 首帧加速度不是 1 g：不能初始化姿态，也不能计入收敛样本。
            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 0.0);

            fake_->set_raw_accel(0, 0, FakeMpu6500::kGravityRaw);
            for (int sample = 1; sample < 5; ++sample) {
                ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                          hardware_interface::return_type::OK);
                EXPECT_DOUBLE_EQ(states[10].get_value(), 0.0) << "sample " << sample;
            }
            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 1.0);
        }

        TEST_F(SensorInterfaceTest, OrientationValidSurvivesLinearAccelerationAfterConvergence) {
            auto info = make_hardware_info(fake_->path(), "imu0", true);
            info.hardware_parameters["orientation.valid_samples"] = "1";

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            auto states = hardware.export_state_interfaces();

            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 1.0);

            // 2 g 的线性加速度不是重力；已经收敛的姿态仍保持 valid，只跳过本帧校正。
            fake_->set_raw_accel(0, 2 * FakeMpu6500::kGravityRaw, 0);
            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 1.0);
        }

        TEST_F(SensorInterfaceTest, OrientationValidClearsOnPersistentReadFailure) {
            auto info = make_hardware_info(fake_->path(), "imu0", true);
            info.hardware_parameters["orientation.valid_samples"] = "1";

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            auto states = hardware.export_state_interfaces();

            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 1.0);

            fake_->write_file("in_anglvel_x_raw", "");
            for (int attempt = 1; attempt <= 3; ++attempt) {
                EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                          hardware_interface::return_type::OK);
                EXPECT_DOUBLE_EQ(states[10].get_value(), 1.0);
            }
            EXPECT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::ERROR);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 0.0);
        }

        TEST_F(SensorInterfaceTest, CleanupClearsOrientationValid) {
            auto info = make_hardware_info(fake_->path(), "imu0", true);
            info.hardware_parameters["orientation.valid_samples"] = "1";

            MPU6500SensorInterface hardware;
            ASSERT_TRUE(init_and_configure(hardware, info));
            auto states = hardware.export_state_interfaces();
            ASSERT_EQ(hardware.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01)),
                      hardware_interface::return_type::OK);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 1.0);

            ASSERT_EQ(hardware.on_cleanup(rclcpp_lifecycle::State()),
                      hardware_interface::CallbackReturn::SUCCESS);
            EXPECT_DOUBLE_EQ(states[10].get_value(), 0.0);
        }

        TEST_F(SensorInterfaceTest, RejectsInvalidValidSamples) {
            auto info = make_hardware_info(fake_->path(), "imu0", true);
            info.hardware_parameters["orientation.valid_samples"] = "0";

            MPU6500SensorInterface hardware;
            EXPECT_EQ(hardware.on_init(info), hardware_interface::CallbackReturn::ERROR);
        }

    } // namespace
} // namespace mpu6500_ros2_control
