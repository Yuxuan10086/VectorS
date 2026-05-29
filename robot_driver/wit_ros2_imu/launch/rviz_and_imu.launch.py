from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # WIT IMU 驱动节点
    # 现在真正支持通过 parameters 传入串口和波特率
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        output='screen',
        parameters=[
            {'port': '/dev/imu_usb'},
            {'baud': 9600}
        ],
        # 如果你希望把话题重映射成 /imu/data，可以取消下面一行的注释
        # remappings=[('/imu/data_raw', '/imu/data')]
    )

    # RViz 可视化节点（默认关闭，如需启用请去掉注释）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        imu_node,
        # rviz_node
    ])

