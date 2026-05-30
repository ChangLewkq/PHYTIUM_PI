#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument('host', default_value='0.0.0.0')
    port_arg = DeclareLaunchArgument('port', default_value='5000')
    yolo_arg = DeclareLaunchArgument('yolo_receiver_url', default_value='http://192.168.43.163:8080')
    scan_arg = DeclareLaunchArgument('scan_topic', default_value='/scan')
    radar_flip_lr_arg = DeclareLaunchArgument('radar_flip_lr', default_value='true')

    return LaunchDescription([
        host_arg,
        port_arg,
        yolo_arg,
        scan_arg,
        radar_flip_lr_arg,
        Node(
            package='phytium_web',
            executable='web_server',
            name='web_server',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'yolo_receiver_url': LaunchConfiguration('yolo_receiver_url'),
                'cmd_topic': '/cmd_vel_web',
                'max_linear': 0.20,
                'max_angular': 0.60,
                'scan_topic': LaunchConfiguration('scan_topic'),
                'radar_max_range': 4.0,
                'radar_min_range': 0.05,
                'radar_downsample': 2,
                'radar_front_sector_deg': 30.0,
                'radar_side_sector_deg': 60.0,
                'radar_flip_lr': LaunchConfiguration('radar_flip_lr'),
            }]
        ),
    ])
