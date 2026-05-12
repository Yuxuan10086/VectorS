# 同 test.launch.py；无 TTY 时标定后不能键盘交互，见包 README。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _resolve_cfg_path(params_file_launch_arg: str) -> str:
    p = (params_file_launch_arg or '').strip()
    if p:
        if not os.path.isabs(p):
            p = os.path.normpath(os.path.join(os.getcwd(), p))
        if os.path.isfile(p):
            return os.path.abspath(p)
    env_p = os.environ.get('SCARA_ARM_PARAMS_FILE', '').strip()
    if env_p and os.path.isfile(env_p):
        return os.path.abspath(env_p)
    pkg = get_package_share_directory('scara_arm')
    return os.path.join(pkg, 'config', 'scara_arm.yaml')


def _launch_setup(context, *_args, **_kwargs):
    override = LaunchConfiguration('params_file').perform(context)
    cfg = _resolve_cfg_path(override)
    return [
        Node(
            package='scara_arm',
            executable='test',
            name='scara_arm_test',
            output='screen',
            parameters=[cfg],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='参数 YAML 路径；留空则用 SCARA_ARM_PARAMS_FILE 或安装目录默认配置',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
