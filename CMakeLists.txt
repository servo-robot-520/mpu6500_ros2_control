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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 顶层源码聚合构建（CLion）直接使用本构建树中的 common_cpp target；
# 普通 colcon/独立 CMake 构建则导入已安装的 common_cpp。
if (NOT ROS2_WS_SOURCE_BUILD)
    find_package(common_cpp REQUIRED)
endif ()

# 核心库：Linux IIO sysfs 读取与姿态互补滤波，不依赖 ROS，可单独复用。
add_library(mpu6500_driver SHARED
        src/mpu6500_iio_device.cpp
        src/orientation_filter.cpp)
target_include_directories(
        mpu6500_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ros2_control SensorInterface 插件。
add_library(mpu6500_ros2_control SHARED src/mpu6500_sensor_interface.cpp)
target_include_directories(
        mpu6500_ros2_control PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(
        mpu6500_ros2_control PUBLIC mpu6500_driver
        hardware_interface::hardware_interface
        pluginlib::pluginlib rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle)

if (ROS2_WS_SOURCE_BUILD)
    if (NOT TARGET common_cpp_logging_interface
            OR NOT TARGET common_cpp_logging
            OR NOT TARGET common_cpp_ros_logging)
        message(FATAL_ERROR
                "源码聚合构建需要 common_cpp 的 target。"
                "请在顶层工作区中先添加 common_cpp，再添加 mpu6500_ros2_control。")
    endif ()
    target_link_libraries(
            mpu6500_ros2_control PUBLIC common_cpp_logging_interface
            common_cpp_logging common_cpp_ros_logging)
else ()
    target_link_libraries(
            mpu6500_ros2_control PUBLIC
            common_cpp::common_cpp_logging_interface
            common_cpp::common_cpp_logging
            common_cpp::common_cpp_ros_logging)
endif ()

pluginlib_export_plugin_description_file(
        hardware_interface mpu6500_ros2_control_plugin_description.xml)

install(
        TARGETS mpu6500_driver mpu6500_ros2_control
        EXPORT mpu6500_ros2_controlTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(
        FILES README.md README.zh-CN.md
        DESTINATION share/${PROJECT_NAME})
install(
        FILES mpu6500_ros2_control_plugin_description.xml
        DESTINATION share/${PROJECT_NAME})

ament_export_targets(mpu6500_ros2_controlTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib
        common_cpp)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_orientation_filter test/test_orientation_filter.cpp)
    target_link_libraries(test_orientation_filter mpu6500_driver)

    ament_add_gtest(test_iio_device test/test_iio_device.cpp)
    target_include_directories(test_iio_device PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(test_iio_device mpu6500_driver)

    ament_add_gtest(test_sensor_interface test/test_sensor_interface.cpp)
    target_include_directories(test_sensor_interface PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(test_sensor_interface mpu6500_ros2_control)

    # 构建树中的插件测试需要一份模拟的 ament index，否则 pluginlib 只能看到
    # /opt/ros 下已安装的插件。
    pluginlib_enable_plugin_testing(
            CMAKE_TARGET_VAR mpu6500_pluginlib_test_target
            AMENT_PREFIX_PATH_VAR mpu6500_pluginlib_test_path
            PLUGIN_CATEGORY hardware_interface
            PLUGIN_DESCRIPTIONS mpu6500_ros2_control_plugin_description.xml
            PLUGIN_LIBRARIES mpu6500_ros2_control)
    ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp
            APPEND_ENV AMENT_PREFIX_PATH=${mpu6500_pluginlib_test_path})
    add_dependencies(test_plugin_loading "${mpu6500_pluginlib_test_target}")
    target_link_libraries(test_plugin_loading mpu6500_ros2_control pluginlib::pluginlib)
endif ()

ament_package()
