from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="stop",
            description="cmd_vel_mux mode: web/keyboard/follow/nav/auto/stop"
        ),

        Node(
            package="phytium_car_cpp",
            executable="cmd_vel_mux_cpp",
            name="cmd_vel_mux",
            output="screen",
            parameters=[{
                "mode": mode,

                "web_topic": "/cmd_vel_web",
                "keyboard_topic": "/cmd_vel_keyboard",
                "follow_topic": "/cmd_vel_follow",
                "nav_topic": "/cmd_vel_nav",

                "estop_topic": "/cmd_vel_mux/estop",
                "mode_cmd_topic": "/cmd_vel_mux/mode_cmd",

                "output_topic": "/cmd_vel_raw",
                "active_source_topic": "/cmd_vel_mux/active_source",

                "web_timeout_sec": 0.80,
                "keyboard_timeout_sec": 0.50,
                "follow_timeout_sec": 0.80,
                "nav_timeout_sec": 0.80,

                "rate": 20.0,

                "max_linear": 0.20,
                "max_reverse": 0.08,
                "max_angular": 0.70,

                "debug": False,
                "auto_mode_on_web_cmd": False,

                "auto_priority": "keyboard,web,follow,nav",
            }],
        ),
    ])
