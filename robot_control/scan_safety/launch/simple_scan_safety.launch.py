"""Launch simple scan-based cmd_vel filter (optional inclusion from higher-level bringup)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_topic', default_value='scan',
            description='LaserScan subscription topic.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_in', default_value='cmd_vel_raw',
            description='Input velocity (e.g. teleop); filtered output avoids obstacles.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_out', default_value='cmd_vel',
            description='Output to chassis driver.',
        ),
        DeclareLaunchArgument(
            'enabled', default_value='true',
            description='When false, passthrough cmd_vel_in to cmd_vel_out.',
        ),
        DeclareLaunchArgument(
            'min_distance_m', default_value='0.35',
            description='Stop forward motion if obstacle closer than this (m) in forward sector.',
        ),
        Node(
            package='scan_safety',
            executable='simple_scan_safety',
            name='simple_scan_safety',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_vel_in': LaunchConfiguration('cmd_vel_in'),
                'cmd_vel_out': LaunchConfiguration('cmd_vel_out'),
                'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                'min_distance_m': ParameterValue(
                    LaunchConfiguration('min_distance_m'), value_type=float),
            }],
        ),
    ])
