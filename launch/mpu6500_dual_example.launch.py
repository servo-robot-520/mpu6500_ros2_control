import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Minimal example: two MPU6500 IMUs (imu0 on SPI3-CS0, imu1 on SPI3-CS1) via ros2_control."""
    pkg_share = get_package_share_directory("mpu6500_ros2_control")
    xacro_file = os.path.join(pkg_share, "urdf", "mpu6500_dual_example.urdf.xacro")
    controllers_file = os.path.join(pkg_share, "config", "mpu6500_controllers.yaml")

    imu0_iio = LaunchConfiguration("imu0_iio_device")
    imu1_iio = LaunchConfiguration("imu1_iio_device")
    thread_priority = LaunchConfiguration("thread_priority")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file, " ",
            "imu0_iio_device:=", imu0_iio, " ",
            "imu1_iio_device:=", imu1_iio,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "thread_priority": ParameterValue(thread_priority, value_type=int),
            },
            controllers_file,
        ],
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["imu_sensor_broadcaster_imu0", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["imu_sensor_broadcaster_imu1", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "imu0_iio_device", default_value="/dev/mpu6500-0",
            description="IIO device (or stable udev alias) for imu0 (SPI3 CS0)."),
        DeclareLaunchArgument(
            "imu1_iio_device", default_value="/dev/mpu6500-1",
            description="IIO device (or stable udev alias) for imu1 (SPI3 CS1)."),
        DeclareLaunchArgument(
            "thread_priority", default_value="50",
            description="controller_manager RT thread priority (0 disables SCHED_FIFO)."),
        robot_state_publisher,
        controller_manager,
        TimerAction(period=4.0, actions=spawners),
    ])
