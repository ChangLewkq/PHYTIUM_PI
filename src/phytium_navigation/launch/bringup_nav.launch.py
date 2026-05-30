#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nav2 low CPU navigation bringup wrapper."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('phytium_navigation')

    source_map_file = '/home/user/phytium_ws/src/phytium_navigation/maps/my_map.yaml'
    installed_map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    default_map_file = source_map_file if os.path.exists(source_map_file) else installed_map_file

    source_params = '/home/user/phytium_ws/src/phytium_navigation/config/nav2_params_low_cpu.yaml'
    installed_params = os.path.join(pkg_dir, 'config', 'nav2_params_low_cpu.yaml')
    fallback_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    default_params_file = (
        source_params if os.path.exists(source_params)
        else installed_params if os.path.exists(installed_params)
        else fallback_params
    )

    source_bt_xml = '/home/user/phytium_ws/src/phytium_navigation/config/simple_navigate.xml'
    installed_bt_xml = os.path.join(pkg_dir, 'config', 'simple_navigate.xml')
    default_bt_xml = source_bt_xml if os.path.exists(source_bt_xml) else installed_bt_xml

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Absolute map yaml file for Nav2'
    )

    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml',
        default_value=default_bt_xml,
        description='Behavior tree xml file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Nav2 params yaml file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    return LaunchDescription([
        map_arg,
        bt_xml_arg,
        params_file_arg,
        use_sim_time_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'bt_xml': LaunchConfiguration('bt_xml'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        ),
    ])
