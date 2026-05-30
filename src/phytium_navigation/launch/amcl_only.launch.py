#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('phytium_navigation')

    source_params = '/home/user/phytium_ws/src/phytium_navigation/config/nav2_params_low_cpu.yaml'
    installed_params = os.path.join(pkg_dir, 'config', 'nav2_params_low_cpu.yaml')
    default_params_file = source_params if os.path.exists(source_params) else installed_params

    source_map = '/home/user/phytium_ws/src/phytium_navigation/maps/my_map.yaml'
    installed_map = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    default_map_file = source_map if os.path.exists(source_map) else installed_map

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='AMCL params file'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Map yaml file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time'
    )

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    scan_amcl_delay = ExecuteProcess(
        cmd=[
            'python3',
            '/home/user/phytium_ws/src/phytium_navigation/scripts/scan_amcl_delay.py',
            '--ros-args',
            '-p', 'input_topic:=/scan',
            '-p', 'output_topic:=/scan_amcl',
            '-p', 'delay_sec:=0.08',
            '-p', 'frame_id:=laser_link',
        ],
        output='screen'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
            'topic_name': 'map',
            'frame_id': 'map',
        }]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': '/scan_amcl',
                'tf_broadcast': True,
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_footprint',
                'transform_tolerance': 1.0,
                'update_min_d': 0.0,
                'update_min_a': 0.0,
            }
        ]
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    return LaunchDescription([
        params_file_arg,
        map_arg,
        use_sim_time_arg,

        LogInfo(msg=['[amcl-only] params: ', params_file]),
        LogInfo(msg=['[amcl-only] map: ', map_file]),
        LogInfo(msg='[amcl-only] only scan_amcl_delay + map_server + amcl'),

        scan_amcl_delay,
        map_server,
        amcl,
        lifecycle_manager_localization,
    ])
