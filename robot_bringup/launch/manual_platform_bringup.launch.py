#!/usr/bin/env python3
"""
manual_platform_bringup.launch.py

手动控制 + Platform 启动文件（用于调试/手动操作）。

现在启动：
- 通过 robot_platform/launch/platform.launch.py 启动 IMU + 静态 TF + robot_platform 节点
- 最新的网页控制器后台（web_control_panel / FastAPI + WebSocket）

使用方法：
  ros2 launch robot_bringup manual_platform_bringup.launch.py

启动后浏览器访问 http://<机器人IP>:8080 即可使用现代网页控制面板。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 引用核心 Platform Launch（已包含 IMU + TF + platform_node）
    robot_platform_share = get_package_share_directory('robot_platform')
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_platform_share, 'launch', 'platform.launch.py')
        ),
        # arm_auto_calibrate 参数已移除，不再支持启动时自动标定
        # 如需标定，请在启动后通过网页或 ros2 action send_goal /arm/calibrate 调用
    )

    # 启动最新的网页控制器后台（FastAPI + WebSocket 版）
    manual_control_share = get_package_share_directory('manual_control')
    web_panel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manual_control_share, 'launch', 'web_panel.launch.py')
        )
    )

    return LaunchDescription([
        platform_launch,
        web_panel_launch,
    ])
