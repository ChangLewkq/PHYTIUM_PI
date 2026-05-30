#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('phytium_navigation')

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

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Nav2 params yaml file'
    )

    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml',
        default_value=default_bt_xml,
        description='Behavior tree xml file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('cmd_vel', '/cmd_vel_nav'),
        ]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'global_frame': 'map',
                'robot_base_frame': 'base_footprint',
                'odom_topic': '/odom',
                'default_bt_xml_filename': bt_xml,
                'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_follow_path_action_bt_node',
                ],
            }
        ]
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
            ],
        }]
    )

    return LaunchDescription([
        params_file_arg,
        bt_xml_arg,
        use_sim_time_arg,

        LogInfo(msg=['[nav-only] params: ', params_file]),
        LogInfo(msg=['[nav-only] bt xml: ', bt_xml]),
        LogInfo(msg='[nav-only] start planner/controller/bt only'),

        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager_navigation,
    ])
