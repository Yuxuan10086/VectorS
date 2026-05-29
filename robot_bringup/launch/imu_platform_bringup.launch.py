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

    # WIT IMU 驱动节点
    # 注意：使用前请确保已执行 bind_usb.sh（或手动创建 /dev/imu_usb 软链接）
    # 默认发布话题：/imu/data_raw（platform 默认订阅此话题）
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        output='screen',
        parameters=[
            {'port': '/dev/imu_usb'},
            {'baud': 9600}
        ],
        # 如需重映射话题，可取消下面注释：
        # remappings=[('/imu/data_raw', '/imu/data')]
    )

    # 静态 TF：base_link → imu_link
    # 根据实际安装位置（平移 + 旋转）修改参数。
    # 当前默认：IMU 安装在 base_link 正上方 0.12 米处，无旋转（roll/pitch/yaw 均为 0）
    # 格式：x y z yaw pitch roll parent child   （单位：米，弧度）
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0.12', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    # 核心平台节点（包含 DiffDriveChassis + RobotArm）
    # arm_auto_calibrate 默认关闭；如需开机自动标定可改为 True
    platform_node = Node(
        package='robot_platform',
        executable='platform_node',
        name='robot_platform',
        output='screen',
        parameters=[
            chassis_yaml,
            arm_yaml,
            topics_yaml,
            {'arm_auto_calibrate': False}
        ],
    )

    return LaunchDescription([
        imu_node,
        static_tf_imu,
        platform_node,
    ])
