from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/phytium_d6_lidar"),
        DeclareLaunchArgument("baudrate", default_value="230400"),

        Node(
            package="phytium_car_cpp",
            executable="d6_lidar_cpp",
            name="d6_lidar",
            output="screen",
            parameters=[{
                "port": port,
                "baudrate": baudrate,
                "frame_id": "laser_link",
                "topic": "/scan",
                "publish_hz": 10.0,
                "read_period": 0.005,
                "bins": 360,
                "range_min": 0.02,
                "range_max": 15.0,
                "min_points_before_publish": 120,
                "publish_empty_scan": True,
            }],
        )
    ])
