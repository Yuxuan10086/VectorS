from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    chassis_yaml = os.path.join(
        get_package_share_directory('chassis'), 'config', 'chassis.yaml')
    arm_yaml = os.path.join(
        get_package_share_directory('scara_arm'), 'config', 'scara_arm.yaml')
    topics_yaml = os.path.join(
        get_package_share_directory('robot_platform'), 'config', 'platform_topics.yaml')

    return LaunchDescription([
        Node(
            package='robot_platform',
            executable='platform_node',
            name='robot_platform',
            output='screen',
            parameters=[chassis_yaml, arm_yaml, topics_yaml],
        ),
    ])
