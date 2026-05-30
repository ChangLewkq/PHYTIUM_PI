from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/phytium_stm32"),
        DeclareLaunchArgument("baud", default_value="115200"),

        Node(
            package="phytium_car_cpp",
            executable="uart_bridge_cpp",
            name="uart_bridge",
            output="screen",
            parameters=[{
                "port": port,
                "baud": baud,
                "write_period": 0.05,
                "read_period": 0.01,
                "meters_per_tick": 0.000137088,
                "wheel_base": 0.180,
                "base_frame": "base_footprint",
                "odom_frame": "odom",
                "imu_frame": "imu_link",
                "publish_tf": True,
                "cmd_timeout": 0.5,
                "swap_lr": False,
                "left_sign": 1,
                "right_sign": 1,
            }],
        )
    ])
