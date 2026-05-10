from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('arm_collision_calib')
    cfg = os.path.join(pkg, 'config', 'arm_collision.yaml')
    return LaunchDescription([
        Node(
            package='arm_collision_calib',
            executable='collision_calib_node',
            name='collision_calib',
            output='screen',
            parameters=[cfg],
        ),
    ])
