#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bringup_all.launch.py

飞腾派 ROS2 智能小车总启动文件。

默认启动：
1. phytium_car_bringup/bringup_base.launch.py
   - URDF / robot_state_publisher
   - joint_state_publisher
   - uart_bridge
   - D6 雷达

2. phytium_vision/cmd_vel_mux.launch.py
   - 速度仲裁，默认 mode:=stop，防止启动后误动
   - 输出 /cmd_vel_raw，不直接进底盘

3. phytium_vision/safety_guard.launch.py
   - 输入 /cmd_vel_raw
   - 订阅 /scan、/perception/target、/cmd_vel_mux/active_source、/safety/estop
   - 输出最终安全速度 /cmd_vel
   - D6 雷达相对车头后移 15cm，已在 safety_guard 中补偿

4. phytium_web/bringup_web.launch.py
   - Web 总控台
   - 飞腾派性能面板
   - RTX3050 推理面板
   - D6 雷达点云显示

可选启动：
- use_vision:=true   启动 D435i + rgb_sender + target_depth_follower
- use_slam:=true     启动 slam_toolbox 建图
- use_nav:=true      启动 phytium_navigation/bringup_nav.launch.py

注意：
1. use_slam 和 use_nav 不建议同时开启。
2. RTX3050 端 yolo_bbox_server.py 仍需在 Windows 笔记本上单独启动。
3. cmd_vel_mux 默认 stop，确认安全后再切换 mode:=web / follow / nav。
4. 正式控制链路：
   /cmd_vel_web / /cmd_vel_follow / /cmd_vel_nav
      -> cmd_vel_mux
      -> /cmd_vel_raw
      -> safety_guard
      -> /cmd_vel
      -> uart_bridge
      -> STM32
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_py_launch(package_name, rel_path, launch_arguments=None, condition=None):
    pkg_dir = get_package_share_directory(package_name)
    launch_file = os.path.join(pkg_dir, rel_path)

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition
    )


def generate_launch_description():
    # ========== launch arguments ==========
    stm32_port_arg = DeclareLaunchArgument(
        'stm32_port',
        default_value='/dev/phytium_stm32',
        description='STM32 chassis serial port'
    )

    radar_port_arg = DeclareLaunchArgument(
        'radar_port',
        default_value='/dev/phytium_d6_lidar',
        description='D6 lidar serial port'
    )

    inference_host_arg = DeclareLaunchArgument(
        'inference_host',
        default_value='192.168.43.163',
        description='RTX3050 inference host IP'
    )

    yolo_receiver_url_arg = DeclareLaunchArgument(
        'yolo_receiver_url',
        default_value='http://192.168.43.163:8080',
        description='RTX3050 YOLO web/API base URL'
    )

    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='5000',
        description='Web dashboard port'
    )

    mux_mode_arg = DeclareLaunchArgument(
        'mux_mode',
        default_value='stop',
        description='cmd_vel_mux mode: stop/web/keyboard/follow/nav/auto'
    )

    radar_flip_lr_arg = DeclareLaunchArgument(
        'radar_flip_lr',
        default_value='true',
        description='Flip radar left/right display in web dashboard'
    )

    use_base_arg = DeclareLaunchArgument(
        'use_base',
        default_value='true',
        description='Launch base chassis, URDF and D6 lidar'
    )

    use_mux_arg = DeclareLaunchArgument(
        'use_mux',
        default_value='true',
        description='Launch cmd_vel_mux'
    )

    use_safety_arg = DeclareLaunchArgument(
        'use_safety',
        default_value='true',
        description='Launch safety_guard. Strongly recommended for real robot.'
    )

    use_web_arg = DeclareLaunchArgument(
        'use_web',
        default_value='true',
        description='Launch web dashboard'
    )

    use_vision_arg = DeclareLaunchArgument(
        'use_vision',
        default_value='false',
        description='Launch D435i vision client and target follower'
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Launch slam_toolbox mapping'
    )

    use_nav_arg = DeclareLaunchArgument(
        'use_nav',
        default_value='false',
        description='Launch navigation bringup'
    )

    # ========== modules ==========
    base_launch = include_py_launch(
        'phytium_car_bringup',
        'launch/bringup_base.launch.py',
        launch_arguments={
            'stm32_port': LaunchConfiguration('stm32_port'),
            'radar_port': LaunchConfiguration('radar_port'),
        },
        condition=IfCondition(LaunchConfiguration('use_base'))
    )

    mux_launch = include_py_launch(
        'phytium_vision',
        'launch/cmd_vel_mux.launch.py',
        launch_arguments={
            'mode': LaunchConfiguration('mux_mode'),
        },
        condition=IfCondition(LaunchConfiguration('use_mux'))
    )

    safety_launch = include_py_launch(
        'phytium_vision',
        'launch/safety_guard.launch.py',
        launch_arguments={
            'input_cmd_topic': '/cmd_vel_raw',
            'output_cmd_topic': '/cmd_vel',
            'scan_topic': '/scan',
        },
        condition=IfCondition(LaunchConfiguration('use_safety'))
    )

    web_launch = include_py_launch(
        'phytium_web',
        'launch/bringup_web.launch.py',
        launch_arguments={
            'port': LaunchConfiguration('web_port'),
            'yolo_receiver_url': LaunchConfiguration('yolo_receiver_url'),
            'radar_flip_lr': LaunchConfiguration('radar_flip_lr'),
        },
        condition=IfCondition(LaunchConfiguration('use_web'))
    )

    vision_launch = include_py_launch(
        'phytium_vision',
        'launch/vision_client.launch.py',
        launch_arguments={
            'inference_host': LaunchConfiguration('inference_host'),
        },
        condition=IfCondition(LaunchConfiguration('use_vision'))
    )

    slam_launch = include_py_launch(
        'phytium_car_bringup',
        'launch/slam_mapping.launch.py',
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    nav_launch = include_py_launch(
        'phytium_navigation',
        'launch/bringup_nav.launch.py',
        condition=IfCondition(LaunchConfiguration('use_nav'))
    )

    return LaunchDescription([
        stm32_port_arg,
        radar_port_arg,
        inference_host_arg,
        yolo_receiver_url_arg,
        web_port_arg,
        mux_mode_arg,
        radar_flip_lr_arg,

        use_base_arg,
        use_mux_arg,
        use_safety_arg,
        use_web_arg,
        use_vision_arg,
        use_slam_arg,
        use_nav_arg,

        LogInfo(msg='[bringup_all] Starting Phytium ROS2 robot system...'),
        LogInfo(msg='[bringup_all] Control chain: mux -> /cmd_vel_raw -> safety_guard -> /cmd_vel -> uart_bridge.'),
        LogInfo(msg='[bringup_all] Default mux mode is stop. Use ros2 param set /cmd_vel_mux mode web/follow/nav after safety check.'),

        base_launch,
        mux_launch,
        safety_launch,
        web_launch,
        vision_launch,
        slam_launch,
        nav_launch,
    ])
