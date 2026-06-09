#!/usr/bin/env python3
"""
platform.launch.py

Robot Platform 核心启动文件（推荐直接使用）。

负责启动：
- WIT IMU 驱动节点（/imu/data_raw，frame_id=base_link）
- robot_platform 节点（底盘 + 机械臂 SDK）

所有常用参数均可通过命令行 `xxx:=value` 覆盖。

示例：
  ros2 launch robot_platform platform.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ===================== Launch Arguments =====================
    declare_imu_port = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/imu_usb',
        description='IMU 串口设备路径（需先运行 bind_usb.sh 创建软链接）'
    )

    declare_imu_baud = DeclareLaunchArgument(
        'imu_baud',
        default_value='9600',
        description='IMU 串口波特率'
    )

    # arm_auto_calibrate 已移除（不再支持启动自动标定，请使用 /arm/calibrate Action）

    # ===================== 参数文件加载 =====================
    chassis_yaml = os.path.join(
        get_package_share_directory('chassis'), 'config', 'chassis.yaml')
    arm_yaml = os.path.join(
        get_package_share_directory('scara_arm'), 'config', 'scara_arm.yaml')
    topics_yaml = os.path.join(
        get_package_share_directory('robot_platform'), 'config', 'platform_topics.yaml')

    # ===================== 节点定义 =====================
    # WIT IMU 驱动
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('imu_port')},
            {'baud': LaunchConfiguration('imu_baud')}
        ],
        # 如需重映射 IMU 话题，可取消下面注释
        # remappings=[('/imu/data_raw', '/imu/data')]
    )

    # 核心平台节点
    platform_node = Node(
        package='robot_platform',
        executable='platform_node',
        name='robot_platform',
        output='screen',
        parameters=[
            chassis_yaml,
            arm_yaml,
            topics_yaml,
        ],
    )

    return LaunchDescription([
        declare_imu_port,
        declare_imu_baud,
        imu_node,
        platform_node,
    ])
