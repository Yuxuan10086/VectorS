"""Launch file for the web-based manual control panel."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'web_host',
            default_value='0.0.0.0',
            description='Web control panel listening host (0.0.0.0 for LAN access)'
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Web control panel HTTP port'
        ),
        DeclareLaunchArgument(
            'chassis_cmd_vel_topic',
            default_value='/chassis/cmd_vel',
            description='Twist topic for chassis velocity'
        ),
        DeclareLaunchArgument(
            'arm_joint_command_topic',
            default_value='/arm/joint_command',
            description='JointState topic for arm position commands'
        ),
        DeclareLaunchArgument(
            'linear_speed_x',
            default_value='0.15',
            description='Default max linear speed (m/s) for twist commands'
        ),
        DeclareLaunchArgument(
            'angular_speed_z',
            default_value='0.6',
            description='Default max angular speed (rad/s)'
        ),

        Node(
            package='manual_control',
            executable='web_control_panel',
            name='web_control_panel',
            output='screen',
            parameters=[{
                'web_host': LaunchConfiguration('web_host'),
                'web_port': LaunchConfiguration('web_port'),
                'chassis_cmd_vel_topic': LaunchConfiguration('chassis_cmd_vel_topic'),
                'arm_joint_command_topic': LaunchConfiguration('arm_joint_command_topic'),
                'linear_speed_x': LaunchConfiguration('linear_speed_x'),
                'angular_speed_z': LaunchConfiguration('angular_speed_z'),
            }],
        ),
    ])
