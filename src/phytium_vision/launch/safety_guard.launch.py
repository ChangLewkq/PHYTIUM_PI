#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_cmd_arg = DeclareLaunchArgument(
        "input_cmd_topic",
        default_value="/cmd_vel_raw"
    )

    output_cmd_arg = DeclareLaunchArgument(
        "output_cmd_topic",
        default_value="/cmd_vel"
    )

    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan"
    )

    return LaunchDescription([
        input_cmd_arg,
        output_cmd_arg,
        scan_topic_arg,

        Node(
            package="phytium_car_cpp",
            executable="safety_guard_cpp",
            name="safety_guard",
            output="screen",
            parameters=[{
                "input_cmd_topic": LaunchConfiguration("input_cmd_topic"),
                "output_cmd_topic": LaunchConfiguration("output_cmd_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),

                "active_source_topic": "/cmd_vel_mux/active_source",
                "target_topic": "/perception/target",
                "estop_topic": "/safety/estop",
                "status_topic": "/safety/status",

                "rate_hz": 20.0,
                "cmd_timeout_sec": 0.50,
                "scan_timeout_sec": 0.80,
                "follow_target_timeout_sec": 0.60,

                "max_forward": 0.10,
                "max_reverse": 0.05,
                "max_angular": 0.35,

                # Same physical rule as STM32 ultrasonic layer:
                # robot-front clearance >25cm normal,
                # 10~25cm slow forward,
                # <=10cm block forward.
                #
                # D6 LiDAR is mounted 15cm behind the ultrasonic/front edge.
                # safety_guard compares:
                # front_clearance = lidar_front_min - lidar_to_front_offset
                "enable_scan_guard": True,
                "front_angle_deg": 25.0,
                "lidar_to_front_offset": 0.15,
                "front_min_valid_range": 0.16,
                "slow_distance": 0.25,
                "stop_distance": 0.10,
                "slow_max_forward": 0.08,

                "enable_follow_guard": True,
                "follow_source_name": "follow",
                "stop_follow_when_target_lost": True,

                "debug": False,
                "status_publish_period_sec": 0.20,
            }],
        )
    ])
