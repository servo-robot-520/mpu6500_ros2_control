#include <gtest/gtest.h>

#include <hardware_interface/sensor_interface.hpp>

#include <exception>
#include <pluginlib/class_loader.hpp>

TEST(PluginLoading, LoadsMpu6500SensorInterface) {
    pluginlib::ClassLoader<hardware_interface::SensorInterface> loader(
        "hardware_interface", "hardware_interface::SensorInterface");

    pluginlib::UniquePtr<hardware_interface::SensorInterface> instance;
    try {
        instance = loader.createUniqueInstance(
            "mpu6500_ros2_control/MPU6500SensorInterface");
    }
    catch (const std::exception& error) {
        FAIL() << error.what();
    }
    EXPECT_TRUE(instance != nullptr);
}
