cmake_minimum_required(VERSION 3.16)
project(mpu6500_ros2_control)

option(BUILD_TESTING "构建单元测试" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)

# 核心库：Linux IIO sysfs 读取与姿态互补滤波，不依赖 ROS，可单独复用。
add_library(mpu6500_driver SHARED
        src/mpu6500_iio_device.cpp
        src/orientation_filter.cpp)
target_include_directories(
        mpu6500_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(
        TARGETS mpu6500_driver
        EXPORT mpu6500_ros2_controlTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(mpu6500_ros2_controlTargets HAS_LIBRARY_TARGET)
ament_package()
