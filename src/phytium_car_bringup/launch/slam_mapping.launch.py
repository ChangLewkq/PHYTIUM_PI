#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SLAM 建图启动：延迟启动 slam_toolbox，等待 TF / Odom / Laser 稳定。"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('phytium_car_bringup')
    default_params_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='slam_toolbox parameter file'
    )

    delay_arg = DeclareLaunchArgument(
        'slam_delay',
        default_value='8.0',
        description='Delay seconds before starting slam_toolbox'
    )

    slam_node = TimerAction(
        period=LaunchConfiguration('slam_delay'),
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
            )
        ]
    )

    return LaunchDescription([
        params_file_arg,
        delay_arg,
        slam_node,
    ])