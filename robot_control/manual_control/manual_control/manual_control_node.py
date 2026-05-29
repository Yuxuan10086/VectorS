"""Keyboard manual control node for chassis and arm."""

import os
import select
import sys
import termios
import time
import tty
from typing import BinaryIO

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_control')

        self.declare_parameter('chassis_cmd_vel_topic', '/chassis/cmd_vel')  # 底盘速度发布话题（Twist）
        self.declare_parameter('arm_joint_command_topic', '/arm/joint_command')  # 机械臂关节目标发布话题（JointState）
        self.declare_parameter('arm_joint_names', ['z', 'j1', 'j2'])  # JointState.name 的关节名顺序

        self.declare_parameter('step_z', 1.0)  # 方向键上下每次触发时 z 轴步进量
        self.declare_parameter('step_j1', 1.0)  # 方向键左右每次触发时 j1 步进量
        self.declare_parameter('step_j2', 1.0)  # 逗号/句号每次触发时 j2 步进量
        self.declare_parameter('linear_speed_x', 0.05)  # W/S 对应底盘 x 方向线速度（m/s）
        self.declare_parameter('angular_speed_z', 0.8)  # A/D 对应底盘 z 轴角速度（rad/s）
        self.declare_parameter('key_timeout_sec', 0.8)  # 驱动按键超时（终端按键重复不稳定时建议适当调大）
        self.declare_parameter('publish_hz', 30.0)  # 主循环频率（Hz），用于读键与发布控制

        self.declare_parameter('z_min', -1.0e9)  # z 轴最小允许值（逻辑单位）
        self.declare_parameter('z_max', 1.0e9)  # z 轴最大允许值（逻辑单位）
        self.declare_parameter('j1_min', -1.0e9)  # j1 最小允许值（逻辑单位）
        self.declare_parameter('j1_max', 1.0e9)  # j1 最大允许值（逻辑单位）
        self.declare_parameter('j2_min', -1.0e9)  # j2 最小允许值（逻辑单位）
        self.declare_parameter('j2_max', 1.0e9)  # j2 最大允许值（逻辑单位）

        self.declare_parameter('z_init', 50.0)  # 启动时 z 初始目标值（建议可达区间中位）
        self.declare_parameter('j1_init', 90.0)  # 启动时 j1 初始目标值（建议可达区间中位）
        self.declare_parameter('j2_init', 90.0)  # 启动时 j2 初始目标值（建议可达区间中位）
        # 修改参数示例：
        # ros2 run manual_control manual_control_node --ros-args -p linear_speed_x:=0.05 -p angular_speed_z:=0.6

        chassis_topic = str(self.get_parameter('chassis_cmd_vel_topic').value)
        arm_topic = str(self.get_parameter('arm_joint_command_topic').value)
        self._joint_names = list(self.get_parameter('arm_joint_names').value)
        if len(self._joint_names) != 3:
            self._joint_names = ['z', 'j1', 'j2']

        self._step_z = float(self.get_parameter('step_z').value)
        self._step_j1 = float(self.get_parameter('step_j1').value)
        self._step_j2 = float(self.get_parameter('step_j2').value)
        self._linear_speed = float(self.get_parameter('linear_speed_x').value)
        self._angular_speed = float(self.get_parameter('angular_speed_z').value)
        self._key_timeout = float(self.get_parameter('key_timeout_sec').value)
        publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))

        self._z_min = float(self.get_parameter('z_min').value)
        self._z_max = float(self.get_parameter('z_max').value)
        self._j1_min = float(self.get_parameter('j1_min').value)
        self._j1_max = float(self.get_parameter('j1_max').value)
        self._j2_min = float(self.get_parameter('j2_min').value)
        self._j2_max = float(self.get_parameter('j2_max').value)

        self._z = float(self.get_parameter('z_init').value)
        self._j1 = float(self.get_parameter('j1_init').value)
        self._j2 = float(self.get_parameter('j2_init').value)

        self._pub_twist = self.create_publisher(Twist, chassis_topic, 10)
        self._pub_joint = self.create_publisher(JointState, arm_topic, 10)

        self._last_w = -1.0
        self._last_a = -1.0
        self._last_s = -1.0
        self._last_d = -1.0
        self._input_buffer = ''
        self._last_twist = (0.0, 0.0)
        self._last_twist_pub_time = -1.0
        self._input_stream, self._close_input_stream = self._open_input_stream()
        self._fd = self._input_stream.fileno()
        self._old_term = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

        self._timer = self.create_timer(1.0 / publish_hz, self._on_timer)
        self._print_help(chassis_topic, arm_topic)
        self._publish_arm()

    def destroy_node(self) -> bool:
        self._restore_terminal()
        return super().destroy_node()

    def _restore_terminal(self) -> None:
        if hasattr(self, '_fd') and hasattr(self, '_old_term'):
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
        if getattr(self, '_close_input_stream', False) and hasattr(self, '_input_stream'):
            self._input_stream.close()

    def _open_input_stream(self) -> tuple[BinaryIO, bool]:
        if sys.stdin.isatty():
            return sys.stdin.buffer, False
        try:
            tty_stream = open('/dev/tty', 'rb', buffering=0)
            self.get_logger().info('stdin 非 tty，已切换到 /dev/tty 读取键盘输入')
            return tty_stream, True
        except OSError as exc:
            raise RuntimeError(
                'manual_control_node 需要终端键盘输入（stdin 非 tty 且无法打开 /dev/tty）'
            ) from exc

    def _print_help(self, chassis_topic: str, arm_topic: str) -> None:
        self.get_logger().info(
            f'手动控制启动: 发布底盘[{chassis_topic}], 机械臂[{arm_topic}]\n'
            '方向键: 上/下=>z +/-1, 左/右=>j1 -/+1\n'
            ', 和 .: j2 -/+1\n'
            'W/S: x线速度 +/- ; A/D: 角速度 +/- ; 松开后自动归零\n'
            'Ctrl+C 退出'
        )

    def _publish_arm(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [self._z, self._j1, self._j2]
        self._pub_joint.publish(msg)

    def _apply_arm_delta(self, dz: float, dj1: float, dj2: float) -> None:
        self._z = _clamp(self._z + dz, self._z_min, self._z_max)
        self._j1 = _clamp(self._j1 + dj1, self._j1_min, self._j1_max)
        self._j2 = _clamp(self._j2 + dj2, self._j2_min, self._j2_max)
        self._publish_arm()

    def _update_drive_key(self, key: str, now: float) -> None:
        if key == 'w':
            self._last_w = now
        elif key == 'a':
            self._last_a = now
        elif key == 's':
            self._last_s = now
        elif key == 'd':
            self._last_d = now

    def _handle_key(self, key: str, now: float) -> None:
        if key == 'UP':
            self._apply_arm_delta(self._step_z, 0.0, 0.0)
        elif key == 'DOWN':
            self._apply_arm_delta(-self._step_z, 0.0, 0.0)
        elif key == 'LEFT':
            self._apply_arm_delta(0.0, -self._step_j1, 0.0)
        elif key == 'RIGHT':
            self._apply_arm_delta(0.0, self._step_j1, 0.0)
        elif key in {',', '，', '<'}:
            self._apply_arm_delta(0.0, 0.0, -self._step_j2)
        elif key in {'.', '。', '>'}:
            self._apply_arm_delta(0.0, 0.0, self._step_j2)
        elif key in {'w', 'a', 's', 'd'}:
            self._update_drive_key(key, now)

    def _try_read_stdin(self) -> None:
        while True:
            ready, _, _ = select.select([self._fd], [], [], 0.0)
            if not ready:
                break
            chunk = os.read(self._fd, 128)
            if not chunk:
                break
            self._input_buffer += chunk.decode('utf-8', errors='ignore')

    def _consume_key_events(self) -> list[str]:
        keys: list[str] = []
        idx = 0
        n = len(self._input_buffer)
        while idx < n:
            ch = self._input_buffer[idx]
            if ch == '\x1b':
                if idx + 2 >= n:
                    break
                if self._input_buffer[idx + 1] == '[':
                    code = self._input_buffer[idx + 2]
                    if code == 'A':
                        keys.append('UP')
                        idx += 3
                        continue
                    if code == 'B':
                        keys.append('DOWN')
                        idx += 3
                        continue
                    if code == 'C':
                        keys.append('RIGHT')
                        idx += 3
                        continue
                    if code == 'D':
                        keys.append('LEFT')
                        idx += 3
                        continue
                idx += 1
                continue

            keys.append(ch.lower())
            idx += 1

        self._input_buffer = self._input_buffer[idx:]
        return keys

    def _publish_twist(self, now: float) -> None:
        active_w = now - self._last_w <= self._key_timeout
        active_s = now - self._last_s <= self._key_timeout
        active_a = now - self._last_a <= self._key_timeout
        active_d = now - self._last_d <= self._key_timeout

        msg = Twist()
        if active_w and active_s:
            # 同轴冲突时按最近一次按下的方向，避免速度在 0 附近抖动
            msg.linear.x = self._linear_speed if self._last_w >= self._last_s else -self._linear_speed
        elif active_w:
            msg.linear.x = self._linear_speed
        elif active_s:
            msg.linear.x = -self._linear_speed

        if active_a and active_d:
            # 同轴冲突时按最近一次按下的方向，避免角速度在 0 附近抖动
            msg.angular.z = self._angular_speed if self._last_a >= self._last_d else -self._angular_speed
        elif active_a:
            msg.angular.z = self._angular_speed
        elif active_d:
            msg.angular.z = -self._angular_speed
        current = (msg.linear.x, msg.angular.z)
        changed = current != self._last_twist
        moving = abs(current[0]) > 1e-9 or abs(current[1]) > 1e-9
        keepalive_due = moving and (self._last_twist_pub_time < 0.0 or now - self._last_twist_pub_time >= 0.10)
        should_publish = changed or keepalive_due
        if should_publish:
            self._pub_twist.publish(msg)
            self._last_twist = current
            self._last_twist_pub_time = now

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._try_read_stdin()
        for key in self._consume_key_events():
            self._handle_key(key, now)
        self._publish_twist(now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
