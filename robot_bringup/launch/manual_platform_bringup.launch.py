#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    chassis_yaml = os.path.join(
        get_package_share_directory('chassis'), 'config', 'chassis.yaml')
    arm_yaml = os.path.join(
        get_package_share_directory('scara_arm'), 'config', 'scara_arm.yaml')
    topics_yaml = os.path.join(
        get_package_share_directory('robot_platform'), 'config', 'platform_topics.yaml')

    platform_node = Node(
        package='robot_platform',
        executable='platform_node',
        name='robot_platform',
        output='screen',
        parameters=[chassis_yaml, arm_yaml, topics_yaml, {'arm_auto_calibrate': True}],
    )

    manual_control_node = Node(
        package='manual_control',
        executable='manual_control_node',
        name='manual_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'z_init': 50.0,
            'j1_init': 90.0,
            'j2_init': 90.0,
            'key_timeout_sec': 0.35,
        }],
    )

    return LaunchDescription([
        platform_node,
        manual_control_node,
    ])
