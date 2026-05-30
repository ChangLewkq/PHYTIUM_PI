#!/usr/bin/env python3
"""SLAM建图启动"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ekf_params = os.path.join(get_package_share_directory('phytium_navigation'), 'config', 'ekf.yaml')
    return LaunchDescription([
        Node(package='robot_localization', executable='ekf_node',
             parameters=[ekf_params]),
        TimerAction(period=8.0, actions=[
            Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                 parameters=[{'odom_frame': 'odom', 'base_frame': 'base_footprint',
                              'map_frame': 'map', 'scan_topic': '/scan', 'mode': 'mapping',
                              'use_odometry': False, 'max_laser_range': 15.0,
                              'transform_timeout': 2.0, 'throttle_scans': 1,
                              'minimum_time_interval': 0.5}]),
        ]),
    ])
