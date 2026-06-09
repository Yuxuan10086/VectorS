#!/usr/bin/env python3
"""
imu_platform_bringup.launch.py

IMU + Platform 整机启动入口（推荐用于无额外 UI 的真实机器人场景）。

此文件现在是薄封装，直接引用 robot_platform/launch/platform.launch.py。
所有 IMU、静态 TF、platform_node 的启动逻辑已集中到 platform.launch.py 中。

支持通过命令行参数覆盖（参数定义见 platform.launch.py）：

  ros2 launch robot_bringup imu_platform_bringup.launch.py
  ros2 launch robot_bringup imu_platform_bringup.launch.py arm_auto_calibrate:=true imu_z:=0.15
  ros2 launch robot_bringup imu_platform_bringup.launch.py imu_port:=/dev/ttyUSB0 imu_baud:=115200
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    platform_launch_file = os.path.join(
        get_package_share_directory('robot_platform'),
        'launch',
        'platform.launch.py'
    )

    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(platform_launch_file)
    )

    return LaunchDescription([
        platform_launch,
    ])
