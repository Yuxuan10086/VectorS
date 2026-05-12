# 加载 YAML 并启动 test；子进程无交互终端，无法输入 j1/j2。需要终端交互请用：ros2 run scara_arm test --ros-args --params-file ...
#
# 参数文件解析顺序（便于改 src 下 YAML 后无需 colcon install）：
#   1) launch 参数 params_file 非空
#   2) 环境变量 SCARA_ARM_PARAMS_FILE 指向存在文件
#   3) install/share/scara_arm/config/scara_arm.yaml
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
            description='参数 YAML 路径（绝对或相对 cwd）；留空则用 SCARA_ARM_PARAMS_FILE 或安装目录下默认配置',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
