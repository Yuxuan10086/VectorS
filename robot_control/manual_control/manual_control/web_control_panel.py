"""Web control panel for robot using FastAPI + WebSocket.

Provides browser-based manual control for chassis (Twist) and arm (JointState),
plus mode switching service and blocking move/span actions from robot_platform.
"""

import asyncio
import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from functools import partial
from typing import Any

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState

# 受保护导入 robot_interfaces 的自定义接口（.srv / .action）。
# 如果失败，给出清晰的中文错误提示，指导用户正确构建。
try:
    from robot_interfaces.srv import (
        SetDriveMode,
        GetMotorState,
        GetMotorSpeedLoopPid,
        SetMotorSpeedLoopPid,
    )
    from robot_interfaces.action import ChassisMove, ChassisSpan, ArmCalibrate
except Exception as _e:  # noqa: BLE001
    raise RuntimeError(
        "无法导入 robot_interfaces 的自定义服务/动作接口（SetDriveMode、电机监控、ChassisMove 等）。\n"
        "这通常是因为只单独构建了 manual_control，而未同时构建提供接口的 robot_interfaces。\n\n"
        "正确做法：\n"
        "  cd ~/robot_ws\n"
        "  colcon build --packages-select robot_interfaces robot_platform manual_control\n"
        "  source install/setup.bash\n\n"
        "然后再启动网页控制面板：\n"
        "  ros2 launch manual_control web_panel.launch.py\n\n"
        f"原始错误: {_e}"
    ) from _e

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse

import uvicorn

try:
    from ament_index_python.packages import get_package_share_directory
    HAS_AMENT = True
except Exception:
    HAS_AMENT = False


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _as_bool(val: Any) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).strip().lower() in ('1', 'true', 'yes', 'on')


# 默认电机列表（与 chassis.yaml + scara_arm.yaml 一致）
DEFAULT_MONITOR_MOTOR_IDS = (5, 4, 1, 3, 2)

_GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class PanelFileLogger:
    """可选会话日志（JSON Lines）。

    记录：模式切换、TWIST（含急停/看门狗）、MOVE/Span 指令与反馈、
    IMU 车身朝向、电机监控状态、PID 读写及底盘双轮 PID 应用。
    """

    def __init__(self, enabled: bool, log_dir: str) -> None:
        self._enabled = bool(enabled)
        self._log_dir = os.path.expanduser(log_dir)
        self._lock = threading.Lock()
        self._fp = None
        self.path = ''
        if self._enabled:
            with self._lock:
                self._open_new_file()

    def _open_new_file(self) -> str:
        os.makedirs(self._log_dir, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(self._log_dir, f'web_panel_{stamp}.log')
        self._fp = open(self.path, 'a', encoding='utf-8')
        return self.path

    def reset_session(self) -> tuple[bool, str, int, str]:
        """关闭当前文件、删除目录内全部 web_panel_*.log、新建会话文件。"""
        with self._lock:
            if self._fp is not None:
                self._fp.close()
                self._fp = None

            deleted = 0
            try:
                os.makedirs(self._log_dir, exist_ok=True)
                for name in os.listdir(self._log_dir):
                    if not (name.startswith('web_panel_') and name.endswith('.log')):
                        continue
                    try:
                        os.remove(os.path.join(self._log_dir, name))
                        deleted += 1
                    except OSError as exc:
                        return False, f'删除 {name} 失败: {exc}', deleted, self.path
            except OSError as exc:
                return False, f'无法访问日志目录: {exc}', deleted, ''

            self._enabled = True
            self._open_new_file()
            return True, 'ok', deleted, self.path

    def log(self, event: str, **fields: Any) -> None:
        if not self._enabled or self._fp is None:
            return
        payload = {
            'ts': datetime.now(timezone.utc).isoformat(),
            'mono': round(time.monotonic(), 6),
            'event': event,
            **fields,
        }
        line = json.dumps(payload, ensure_ascii=False, default=str)
        with self._lock:
            self._fp.write(line + '\n')
            self._fp.flush()

    def close(self) -> None:
        if self._fp is not None:
            with self._lock:
                self._fp.close()
                self._fp = None


class WebControlPanel(Node):
    """ROS node + web server bridge for manual teleop."""

    def __init__(self) -> None:
        super().__init__('web_control_panel')

        # Topics & joint config (match robot_platform defaults)
        self.declare_parameter('chassis_cmd_vel_topic', '/chassis/cmd_vel')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('arm_joint_command_topic', '/arm/joint_command')
        self.declare_parameter('arm_joint_names', ['z', 'j1', 'j2'])

        # Web server
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8080)

        # Speeds & steps (tunable via --ros-args)
        self.declare_parameter('linear_speed_x', 0.15)
        self.declare_parameter('angular_speed_z', 0.6)
        self.declare_parameter('step_z', 2.0)
        self.declare_parameter('step_j1', 3.0)
        self.declare_parameter('step_j2', 3.0)

        # Arm limits and inits (recommend setting to reachable from scara_arm.yaml)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 100.0)
        self.declare_parameter('j1_min', 0.0)
        self.declare_parameter('j1_max', 180.0)
        self.declare_parameter('j2_min', 0.0)
        self.declare_parameter('j2_max', 180.0)
        self.declare_parameter('z_init', 50.0)
        self.declare_parameter('j1_init', 90.0)
        self.declare_parameter('j2_init', 90.0)

        # Watchdog: if no twist update for this long, force zero
        self.declare_parameter('twist_watchdog_sec', 0.6)

        # Twist 模式下持续发送频率（满足 robot_platform 300ms 看门狗）
        self.declare_parameter('twist_keepalive_rate_hz', 20.0)

        # 可选会话文件日志（默认关闭）
        self.declare_parameter('panel_log_enable', False)
        self.declare_parameter('panel_log_dir', '~/robot_ws/log/web_panel')

        # Load params
        self._chassis_topic = str(self.get_parameter('chassis_cmd_vel_topic').value)
        self._imu_topic = str(self.get_parameter('imu_topic').value)
        self._arm_topic = str(self.get_parameter('arm_joint_command_topic').value)
        self._joint_names = list(self.get_parameter('arm_joint_names').value)
        if len(self._joint_names) != 3:
            self._joint_names = ['z', 'j1', 'j2']

        self.web_host = str(self.get_parameter('web_host').value)
        self.web_port = int(self.get_parameter('web_port').value)

        self._lin_speed = float(self.get_parameter('linear_speed_x').value)
        self._ang_speed = float(self.get_parameter('angular_speed_z').value)
        self._step_z = float(self.get_parameter('step_z').value)
        self._step_j1 = float(self.get_parameter('step_j1').value)
        self._step_j2 = float(self.get_parameter('step_j2').value)

        self._z_min = float(self.get_parameter('z_min').value)
        self._z_max = float(self.get_parameter('z_max').value)
        self._j1_min = float(self.get_parameter('j1_min').value)
        self._j1_max = float(self.get_parameter('j1_max').value)
        self._j2_min = float(self.get_parameter('j2_min').value)
        self._j2_max = float(self.get_parameter('j2_max').value)

        self._arm_z = float(self.get_parameter('z_init').value)
        self._arm_j1 = float(self.get_parameter('j1_init').value)
        self._arm_j2 = float(self.get_parameter('j2_init').value)

        self._twist_watchdog = float(self.get_parameter('twist_watchdog_sec').value)
        self._twist_keepalive_hz = float(self.get_parameter('twist_keepalive_rate_hz').value)

        self._panel_log_enable = _as_bool(self.get_parameter('panel_log_enable').value)
        self._panel_log_dir = str(self.get_parameter('panel_log_dir').value)

        # Publishers & subscribers
        self._pub_twist = self.create_publisher(Twist, self._chassis_topic, 10)
        self._pub_joint = self.create_publisher(JointState, self._arm_topic, 10)
        self.create_subscription(Imu, self._imu_topic, self._on_imu, 20)

        # Service & Action clients
        self._mode_client = self.create_client(SetDriveMode, '/chassis/set_mode')
        self._move_client = ActionClient(self, ChassisMove, '/chassis/move')
        self._span_client = ActionClient(self, ChassisSpan, '/chassis/span')
        self._calib_client = ActionClient(self, ArmCalibrate, '/arm/calibrate')
        # 电机服务客户端使用可重入回调组，允许在独立轮询线程中阻塞调用
        self._motor_cb_group = ReentrantCallbackGroup()
        self._get_state_client = self.create_client(
            GetMotorState, '/motor/get_state', callback_group=self._motor_cb_group)
        self._get_pid_client = self.create_client(
            GetMotorSpeedLoopPid, '/motor/get_speed_loop_pid', callback_group=self._motor_cb_group)
        self._set_pid_client = self.create_client(
            SetMotorSpeedLoopPid, '/motor/set_speed_loop_pid', callback_group=self._motor_cb_group)

        # Runtime state
        self._current_mode: int = 0  # 0=TWIST, 1=MOVE
        self._last_twist: tuple[float, float] = (0.0, 0.0)
        self._last_twist_time: float = 0.0
        self._active_move_gh = None
        self._active_span_gh = None
        self._active_calib_gh = None

        self._imu_yaw_rad = 0.0
        self._imu_received = False
        self._imu_log_last = 0.0
        self._imu_logged_yaw = 0.0
        self._imu_broadcast_last = 0.0

        self._panel_log = PanelFileLogger(self._panel_log_enable, self._panel_log_dir)
        self._move_feedback_log_last = 0.0
        self._span_feedback_log_last = 0.0
        self._twist_log_last = 0.0
        self._twist_logged: tuple[float, float] = (0.0, 0.0)
        self._monitor_state_log_last: dict[int, float] = {}
        if self._panel_log_enable:
            self.get_logger().info(f'面板会话日志已启用: {self._panel_log.path}')
            self._panel_log.log('session_start', mode=self._current_mode)

        # WebSocket clients (set of WebSocket objects) + event loop for thread-safe push
        self._ws_clients: set[WebSocket] = set()
        self._ws_monitor_clients: set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None

        # 电机监控：Python 侧调度（按 motor_id 开关 + 频率）
        self._monitor_slots: dict[int, dict[str, Any]] = {
            mid: {'enabled': False, 'rate_hz': 15.0, 'last_poll': 0.0}
            for mid in DEFAULT_MONITOR_MOTOR_IDS
        }
        self._monitor_rr_index = 0
        self._monitor_run = True
        self._monitor_thread = threading.Thread(target=self._monitor_poll_loop, daemon=True)
        self._monitor_thread.start()

        # Watchdog + status timer
        self.create_timer(0.1, self._watchdog_and_status)

        # Twist 模式持续 keepalive（无论有无用户输入都循环发，喂饱机器人看门狗）
        if self._twist_keepalive_hz > 0.1:
            self.create_timer(1.0 / self._twist_keepalive_hz, self._twist_keepalive)

        self.get_logger().info(
            f'Web control panel ready. Topics: chassis={self._chassis_topic}, imu={self._imu_topic}, '
            f'arm={self._arm_topic}. '
            f'Control UI http://{self.web_host}:{self.web_port}/  Monitor http://{self.web_host}:{self.web_port}/monitor'
        )
        self._publish_arm()  # initial position

    # ---------------- ROS helpers ----------------

    def set_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    @staticmethod
    def _yaw_from_imu_orientation(qx: float, qy: float, qz: float, qw: float) -> float:
        """四元数转绕 Z 轴航向角（与 robot_platform 一致）。"""
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    def _heading_payload(self) -> dict[str, Any]:
        return {
            'yaw_rad': float(self._imu_yaw_rad),
            'yaw_deg': math.degrees(self._imu_yaw_rad),
            'valid': bool(self._imu_received),
        }

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        if abs(q.w) < 1e-9 and abs(q.x) < 1e-9 and abs(q.y) < 1e-9 and abs(q.z) < 1e-9:
            return

        yaw = self._yaw_from_imu_orientation(q.x, q.y, q.z, q.w)
        self._imu_yaw_rad = yaw
        self._imu_received = True

        now = time.monotonic()
        changed = abs(yaw - self._imu_logged_yaw) > math.radians(0.5)
        if changed or now - self._imu_log_last >= 0.5:
            self._imu_log_last = now
            self._imu_logged_yaw = yaw
            self._plog(
                'imu_heading',
                yaw_rad=round(yaw, 6),
                yaw_deg=round(math.degrees(yaw), 2),
            )

        if changed or now - self._imu_broadcast_last >= 0.2:
            self._imu_broadcast_last = now
            self._safe_broadcast({
                'type': 'imu_heading',
                **self._heading_payload(),
            })

    def _plog_twist(self, linear_x: float, angular_z: float, source: str) -> None:
        now = time.monotonic()
        at_rest = abs(linear_x) < 1e-6 and abs(angular_z) < 1e-6
        was_rest = abs(self._twist_logged[0]) < 1e-6 and abs(self._twist_logged[1]) < 1e-6
        changed = (
            abs(linear_x - self._twist_logged[0]) > 0.01
            or abs(angular_z - self._twist_logged[1]) > 0.01
        )
        if (
            not (at_rest and not was_rest)
            and not changed
            and now - self._twist_log_last < 0.25
        ):
            return
        self._twist_log_last = now
        self._twist_logged = (linear_x, angular_z)
        self._plog('twist', source=source, linear_x=linear_x, angular_z=angular_z)

    def _publish_twist(self, linear_x: float, angular_z: float, *, log_source: str = 'ui') -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._pub_twist.publish(msg)
        self._last_twist = (linear_x, angular_z)
        self._last_twist_time = time.monotonic()
        self._plog_twist(linear_x, angular_z, log_source)
        self._safe_broadcast({
            'type': 'twist',
            'linear_x': linear_x,
            'angular_z': angular_z,
        })

    def _publish_arm(self, z: float | None = None, j1: float | None = None, j2: float | None = None) -> None:
        if z is not None:
            self._arm_z = _clamp(z, self._z_min, self._z_max)
        if j1 is not None:
            self._arm_j1 = _clamp(j1, self._j1_min, self._j1_max)
        if j2 is not None:
            self._arm_j2 = _clamp(j2, self._j2_min, self._j2_max)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [self._arm_z, self._arm_j1, self._arm_j2]
        self._pub_joint.publish(msg)

        self._safe_broadcast({
            'type': 'arm',
            'z': self._arm_z,
            'j1': self._arm_j1,
            'j2': self._arm_j2,
        })

    def _watchdog_and_status(self) -> None:
        now = time.monotonic()
        if (now - self._last_twist_time > self._twist_watchdog and
                self._last_twist != (0.0, 0.0)):
            self._plog(
                'twist_watchdog_stop',
                linear_x=self._last_twist[0],
                angular_z=self._last_twist[1],
            )
            self._publish_twist(0.0, 0.0, log_source='watchdog')

        # periodic status push
        self._safe_broadcast({
            'type': 'status',
            'mode': self._current_mode,
            'arm': {'z': self._arm_z, 'j1': self._arm_j1, 'j2': self._arm_j2},
            'twist': {'linear_x': self._last_twist[0], 'angular_z': self._last_twist[1]},
            'heading': self._heading_payload(),
        })

    def _twist_keepalive(self) -> None:
        """Twist 模式下持续高频发送（含零值），确保机器人 300ms 看门狗不触发。"""
        if self._current_mode == 0:  # 仅 TWIST 模式需要
            lx, az = self._last_twist
            # 直接复用 publish（会更新 last_time，避免 watchdog 误杀）
            self._publish_twist(lx, az, log_source='keepalive')

    # Mode service (blocking wrapper for run_in_executor)
    def _set_mode_blocking(self, mode: int) -> tuple[bool, str]:
        self._plog('set_mode_request', target_mode=int(mode))
        if not self._mode_client.wait_for_service(timeout_sec=3.0):
            self._plog('set_mode_fail', reason='service_unavailable')
            return False, '服务 /chassis/set_mode 不可用（确认 robot_platform 已启动）'
        req = SetDriveMode.Request()
        req.mode = int(mode)
        fut = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done():
            res = fut.result()
            if res.success:
                self._current_mode = mode
            self._safe_broadcast({'type': 'mode', 'mode': self._current_mode})
            self._plog(
                'set_mode_result',
                success=bool(res.success),
                message=res.message or '',
                current_mode=self._current_mode,
            )
            return bool(res.success), res.message or ''
        self._plog('set_mode_fail', reason='timeout')
        return False, '模式切换超时'

    # Action helpers
    def _send_move_blocking(self, distance_m: float, speed_mps: float) -> tuple[bool, str]:
        self._plog('move_send', distance_m=distance_m, speed_mps=speed_mps, active=self._active_goal_snapshot())
        if self._current_mode != 1:
            self._plog('move_reject', reason='not_move_mode')
            return False, '必须先切换到 MOVE 模式'
        if self._chassis_action_busy():
            self._plog('move_reject', reason='action_busy', active=self._active_goal_snapshot())
            return False, '已有 Move/Span 正在执行，请等待到位或点击取消'
        if not self._move_client.wait_for_server(timeout_sec=3.0):
            self._plog('move_reject', reason='server_unavailable')
            return False, 'Action /chassis/move 服务器不可用'
        goal = ChassisMove.Goal()
        goal.distance_m = float(distance_m)
        goal.speed_mps = float(speed_mps)
        send_fut = self._move_client.send_goal_async(goal, feedback_callback=self._on_move_feedback)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=4.0)
        if not send_fut.done():
            self._plog('move_reject', reason='send_timeout')
            return False, '发送 Move Goal 超时'
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self._plog('move_reject', reason='not_accepted', accepted=bool(gh and gh.accepted))
            return False, 'Move Goal 被拒绝（检查当前模式）'
        self._active_move_gh = gh
        result_fut = gh.get_result_async()
        result_fut.add_done_callback(self._on_move_result)
        self._plog('move_accepted', goal_id=str(gh.goal_id))
        return True, 'Move Goal 已接受，开始执行'

    def _on_move_feedback(self, feedback_msg: Any) -> None:
        fb = feedback_msg.feedback
        now = time.monotonic()
        if now - self._move_feedback_log_last >= 0.2:
            self._move_feedback_log_last = now
            self._plog(
                'move_feedback',
                distance_remaining_m=float(fb.distance_remaining_m),
                left_encoder_pos=int(fb.left_encoder_pos),
                right_encoder_pos=int(fb.right_encoder_pos),
            )
        self._safe_broadcast({
            'type': 'move_feedback',
            'distance_remaining_m': float(fb.distance_remaining_m),
            'left_encoder_pos': int(fb.left_encoder_pos),
            'right_encoder_pos': int(fb.right_encoder_pos),
        })

    def _on_move_result(self, fut: Any) -> None:
        self._active_move_gh = None
        try:
            res = fut.result()
            self._plog(
                'move_result',
                status=self._goal_status_name(res.status),
                status_code=int(res.status),
                success=bool(res.result.success),
            )
            self._safe_broadcast({
                'type': 'action_result',
                'action': 'move',
                'success': bool(res.result.success),
                'message': 'Move 完成' if res.result.success else 'Move 失败或被取消',
            })
        except Exception as e:
            self._plog('move_result_error', error=str(e))
            self._safe_broadcast({'type': 'action_result', 'action': 'move', 'success': False, 'message': str(e)})

    def _send_span_blocking(self, angle_rad: float, omega_radps: float) -> tuple[bool, str]:
        self._plog(
            'span_send',
            angle_rad=angle_rad,
            omega_radps=omega_radps,
            active=self._active_goal_snapshot(),
        )
        if self._current_mode != 1:
            self._plog('span_reject', reason='not_move_mode')
            return False, '必须先切换到 MOVE 模式'
        if self._chassis_action_busy():
            self._plog('span_reject', reason='action_busy', active=self._active_goal_snapshot())
            return False, '已有 Move/Span 正在执行，请等待到位或点击取消'
        if not self._span_client.wait_for_server(timeout_sec=3.0):
            self._plog('span_reject', reason='server_unavailable')
            return False, 'Action /chassis/span 服务器不可用'
        goal = ChassisSpan.Goal()
        goal.angle_rad = float(angle_rad)
        goal.omega_radps = float(omega_radps)
        send_fut = self._span_client.send_goal_async(goal, feedback_callback=self._on_span_feedback)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=4.0)
        if not send_fut.done():
            self._plog('span_reject', reason='send_timeout')
            return False, '发送 Span Goal 超时'
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self._plog('span_reject', reason='not_accepted', accepted=bool(gh and gh.accepted))
            return False, 'Span Goal 被拒绝（检查当前模式）'
        self._active_span_gh = gh
        result_fut = gh.get_result_async()
        result_fut.add_done_callback(self._on_span_result)
        self._plog('span_accepted', goal_id=str(gh.goal_id))
        return True, 'Span Goal 已接受，开始执行'

    def _on_span_feedback(self, feedback_msg: Any) -> None:
        fb = feedback_msg.feedback
        now = time.monotonic()
        if now - self._span_feedback_log_last >= 0.2:
            self._span_feedback_log_last = now
            self._plog('span_feedback', angle_remaining_rad=float(fb.angle_remaining_rad))
        self._safe_broadcast({
            'type': 'span_feedback',
            'angle_remaining_rad': float(fb.angle_remaining_rad),
        })

    def _on_span_result(self, fut: Any) -> None:
        self._active_span_gh = None
        try:
            res = fut.result()
            self._plog(
                'span_result',
                status=self._goal_status_name(res.status),
                status_code=int(res.status),
                success=bool(res.result.success),
            )
            self._safe_broadcast({
                'type': 'action_result',
                'action': 'span',
                'success': bool(res.result.success),
                'message': 'Span 完成' if res.result.success else 'Span 失败或被取消',
            })
        except Exception as e:
            self._plog('span_result_error', error=str(e))
            self._safe_broadcast({'type': 'action_result', 'action': 'span', 'success': False, 'message': str(e)})

    # === 新增：ArmCalibrate Action 支持 ===
    def _send_calibrate_blocking(self) -> tuple[bool, str]:
        if not self._calib_client.wait_for_server(timeout_sec=3.0):
            return False, 'Action /arm/calibrate 服务器不可用（确认 robot_platform 已启动）'
        goal = ArmCalibrate.Goal()
        send_fut = self._calib_client.send_goal_async(goal, feedback_callback=self._on_calib_feedback)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=4.0)
        if not send_fut.done():
            return False, '发送 Calibrate Goal 超时'
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return False, 'Calibrate Goal 被拒绝'
        self._active_calib_gh = gh
        result_fut = gh.get_result_async()
        result_fut.add_done_callback(self._on_calib_result)
        return True, '标定已开始（请在界面观察三轴进度）'

    def _on_calib_feedback(self, feedback_msg: Any) -> None:
        fb = feedback_msg.feedback
        self._safe_broadcast({
            'type': 'calib_feedback',
            'completed_axis': fb.completed_axis or '',
            'progress': float(fb.progress),
            'z_done': bool(fb.z_done),
            'j1_done': bool(fb.j1_done),
            'j2_done': bool(fb.j2_done),
            'current_step': fb.current_step or '',
        })

    def _on_calib_result(self, fut: Any) -> None:
        self._active_calib_gh = None
        try:
            res = fut.result()
            self._safe_broadcast({
                'type': 'action_result',
                'action': 'calibrate',
                'success': bool(res.result.success),
                'message': res.result.message or ('标定' + ('成功' if res.result.success else '失败')),
            })
        except Exception as e:
            self._safe_broadcast({'type': 'action_result', 'action': 'calibrate', 'success': False, 'message': str(e)})

    def _cancel_active_actions(self, source: str = 'unknown') -> None:
        self._plog('cancel_actions', source=source, active=self._active_goal_snapshot())
        for gh, name in [(self._active_move_gh, 'move'), (self._active_span_gh, 'span'), (self._active_calib_gh, 'calibrate')]:
            if gh is not None:
                try:
                    cancel_fut = gh.cancel_goal_async()
                    self._plog('cancel_goal_async', action=name, goal_id=str(gh.goal_id))
                    cancel_fut.add_done_callback(
                        lambda f, action=name: self._plog(
                            'cancel_goal_done',
                            action=action,
                            done=f.done(),
                            error=str(f.exception()) if f.done() and f.exception() else '',
                        )
                    )
                except Exception as exc:
                    self._plog('cancel_goal_error', action=name, error=str(exc))
                self._safe_broadcast({'type': 'action_result', 'action': name, 'success': False, 'message': '已取消'})
        self._active_move_gh = None
        self._active_span_gh = None
        self._active_calib_gh = None

    def _plog(self, event: str, **fields: Any) -> None:
        self._panel_log.log(event, mode=self._current_mode, **fields)

    def _reset_panel_log(self) -> tuple[bool, str, int, str]:
        ok, msg, deleted, path = self._panel_log.reset_session()
        if ok:
            self._panel_log_enable = True
            self.get_logger().info(
                f'面板会话日志已重置：删除 {deleted} 个历史文件，新文件 {path}'
            )
            self._panel_log.log(
                'session_start',
                mode=self._current_mode,
                reset=True,
                deleted_files=deleted,
            )
        return ok, msg, deleted, path

    @staticmethod
    def _goal_status_name(status: int) -> str:
        return _GOAL_STATUS_NAMES.get(int(status), f'STATUS_{status}')

    def _active_goal_snapshot(self) -> dict[str, bool]:
        return {
            'move': self._active_move_gh is not None,
            'span': self._active_span_gh is not None,
            'calibrate': self._active_calib_gh is not None,
        }

    def _chassis_action_busy(self) -> bool:
        return self._active_move_gh is not None or self._active_span_gh is not None

    # Thread-safe broadcast from ROS callbacks
    def _safe_broadcast(self, msg: dict[str, Any]) -> None:
        if self._loop is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast(msg), self._loop)
        except Exception:
            pass

    async def _broadcast(self, msg: dict[str, Any]) -> None:
        if not self._ws_clients:
            return
        data = json.dumps(msg, ensure_ascii=False)
        dead: list[WebSocket] = []
        for ws in list(self._ws_clients):
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for d in dead:
            self._ws_clients.discard(d)

    def _safe_broadcast_monitor(self, msg: dict[str, Any]) -> None:
        if self._loop is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast_monitor(msg), self._loop)
        except Exception:
            pass

    async def _broadcast_monitor(self, msg: dict[str, Any]) -> None:
        if not self._ws_monitor_clients:
            return
        data = json.dumps(msg, ensure_ascii=False)
        dead: list[WebSocket] = []
        for ws in list(self._ws_monitor_clients):
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for d in dead:
            self._ws_monitor_clients.discard(d)

    @staticmethod
    def _motor_state_to_dict(state: Any) -> dict[str, Any]:
        return {
            'motor_id': int(state.motor_id),
            'success': bool(state.success),
            'message': str(state.message),
            'bus_voltage_v': float(state.bus_voltage_v),
            'phase_current_ma': int(state.phase_current_ma),
            'flux_mwb': float(state.flux_mwb),
            'phase_resistance_ohm': float(state.phase_resistance_ohm),
            'phase_inductance_mh': float(state.phase_inductance_mh),
            'rpm': int(state.rpm),
            'target_position': int(state.target_position),
            'position': int(state.position),
            'position_error': int(state.position_error),
            'pulse_count': int(state.pulse_count),
            'enabled': bool(state.enabled),
            'arrived': bool(state.arrived),
            'stalled': bool(state.stalled),
            'addr_mode': int(state.addr_mode),
            'error_code': int(state.error_code),
            'error_hint': str(state.error_hint),
        }

    def _monitor_poll_loop(self) -> None:
        """独立轮询线程：与手动「获取一次」相同路径，避免 ROS 定时器线程内 spin 死锁。"""
        while self._monitor_run and rclpy.ok():
            try:
                if not self._ws_monitor_clients:
                    time.sleep(0.1)
                    continue

                enabled_ids = [mid for mid, slot in self._monitor_slots.items() if slot['enabled']]
                if not enabled_ids:
                    time.sleep(0.05)
                    continue

                if not self._get_state_client.wait_for_service(timeout_sec=0.5):
                    time.sleep(0.5)
                    continue

                now = time.monotonic()
                n = len(enabled_ids)
                polled = False
                for _ in range(n):
                    mid = enabled_ids[self._monitor_rr_index % n]
                    self._monitor_rr_index = (self._monitor_rr_index + 1) % n
                    slot = self._monitor_slots[mid]
                    interval = 1.0 / _clamp(float(slot['rate_hz']), 1.0, 30.0)
                    if now - float(slot['last_poll']) < interval:
                        continue

                    slot['last_poll'] = now
                    state = self._get_state_blocking(mid)
                    last_log = self._monitor_state_log_last.get(mid, 0.0)
                    if now - last_log >= 0.5:
                        self._monitor_state_log_last[mid] = now
                        self._plog(
                            'motor_state',
                            channel='monitor',
                            motor_id=mid,
                            success=bool(state.get('success')),
                            rpm=int(state.get('rpm', 0)),
                            position=int(state.get('position', 0)),
                            phase_current_ma=int(state.get('phase_current_ma', 0)),
                            stalled=bool(state.get('stalled', False)),
                            error_code=int(state.get('error_code', 0)),
                        )
                    self._safe_broadcast_monitor({
                        'type': 'motor_state',
                        'state': state,
                    })
                    polled = True
                    break

                time.sleep(0.02 if polled else 0.05)
            except Exception as exc:
                self.get_logger().warn(f'电机监控轮询线程异常: {exc}')
                time.sleep(0.5)

    def _get_state_blocking(self, motor_id: int) -> dict[str, Any]:
        if not self._get_state_client.wait_for_service(timeout_sec=2.0):
            return {
                'motor_id': int(motor_id),
                'success': False,
                'message': '服务 /motor/get_state 不可用',
                'error_hint': '服务 /motor/get_state 不可用',
            }
        req = GetMotorState.Request()
        req.motor_id = int(motor_id) & 0xFF
        fut = self._get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done():
            return {
                'motor_id': int(motor_id),
                'success': False,
                'message': '读取电机状态超时',
                'error_hint': '读取电机状态超时',
            }
        res = fut.result()
        if res is None:
            return {
                'motor_id': int(motor_id),
                'success': False,
                'message': '无响应',
                'error_hint': '无响应',
            }
        return self._motor_state_to_dict(res.state)

    def _get_pid_blocking(self, motor_id: int) -> tuple[bool, str, int, int, int]:
        self._plog('get_pid_request', motor_id=int(motor_id))
        if not self._get_pid_client.wait_for_service(timeout_sec=2.0):
            self._plog('get_pid_fail', motor_id=int(motor_id), reason='service_unavailable')
            return False, '服务 /motor/get_speed_loop_pid 不可用', 0, 0, 0
        req = GetMotorSpeedLoopPid.Request()
        req.motor_id = int(motor_id) & 0xFF
        fut = self._get_pid_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done():
            self._plog('get_pid_fail', motor_id=int(motor_id), reason='timeout')
            return False, '读取 PID 超时', 0, 0, 0
        res = fut.result()
        if res is None or not res.success:
            self._plog(
                'get_pid_fail',
                motor_id=int(motor_id),
                reason=res.message if res else 'no_response',
            )
            return False, res.message if res else '无响应', 0, 0, 0
        self._plog(
            'get_pid_result',
            motor_id=int(motor_id),
            p=int(res.p),
            i=int(res.i),
            d=int(res.d),
        )
        return True, res.message or 'ok', int(res.p), int(res.i), int(res.d)

    def _set_pid_blocking(self, motor_id: int, p: int, i: int, d: int) -> tuple[bool, str, int, int, int]:
        self._plog('set_pid_request', motor_id=int(motor_id), p=int(p), i=int(i), d=int(d))
        if not self._set_pid_client.wait_for_service(timeout_sec=2.0):
            self._plog('set_pid_fail', motor_id=int(motor_id), reason='service_unavailable')
            return False, '服务 /motor/set_speed_loop_pid 不可用', 0, 0, 0
        req = SetMotorSpeedLoopPid.Request()
        req.motor_id = int(motor_id) & 0xFF
        req.p = int(p)
        req.i = int(i)
        req.d = int(d)
        fut = self._set_pid_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done():
            self._plog('set_pid_fail', motor_id=int(motor_id), reason='timeout')
            return False, '设置 PID 超时', 0, 0, 0
        res = fut.result()
        if res is None or not res.success:
            self._plog(
                'set_pid_fail',
                motor_id=int(motor_id),
                reason=res.message if res else 'no_response',
            )
            return False, res.message if res else '无响应', 0, 0, 0
        self._plog(
            'set_pid_result',
            motor_id=int(motor_id),
            p=int(res.p),
            i=int(res.i),
            d=int(res.d),
        )
        return True, res.message or 'ok', int(res.p), int(res.i), int(res.d)

    # ---------------- WebSocket command handling ----------------

    async def handle_ws_command(self, ws: WebSocket, cmd: dict[str, Any]) -> None:
        t = cmd.get('type')
        if t not in ('set_twist', 'ping'):
            self._plog('ws_cmd', cmd_type=t, cmd=cmd)
        try:
            if t == 'set_twist':
                lx = float(cmd.get('linear_x', 0.0))
                az = float(cmd.get('angular_z', 0.0))
                self._publish_twist(lx, az, log_source='ui')

            elif t == 'set_arm':
                z = cmd.get('z')
                j1 = cmd.get('j1')
                j2 = cmd.get('j2')
                self._publish_arm(
                    float(z) if z is not None else None,
                    float(j1) if j1 is not None else None,
                    float(j2) if j2 is not None else None,
                )

            elif t == 'arm_delta':
                dz = float(cmd.get('dz', 0.0))
                dj1 = float(cmd.get('dj1', 0.0))
                dj2 = float(cmd.get('dj2', 0.0))
                self._publish_arm(self._arm_z + dz, self._arm_j1 + dj1, self._arm_j2 + dj2)

            elif t == 'set_mode':
                mode = int(cmd.get('mode', 0))
                prev_mode = self._current_mode
                self._current_mode = mode
                self._safe_broadcast({'type': 'mode', 'mode': mode})
                loop = asyncio.get_running_loop()
                success, message = await loop.run_in_executor(
                    None, partial(self._set_mode_blocking, mode)
                )
                if not success:
                    self._current_mode = prev_mode
                    self._safe_broadcast({'type': 'mode', 'mode': prev_mode})
                await ws.send_text(json.dumps({
                    'type': 'mode_result',
                    'success': success,
                    'message': message,
                    'mode': self._current_mode,
                }, ensure_ascii=False))

            elif t == 'send_move':
                dist = float(cmd.get('distance_m', 0.05))
                spd = float(cmd.get('speed_mps', 0.05))
                loop = asyncio.get_running_loop()
                ok, msg = await loop.run_in_executor(
                    None, partial(self._send_move_blocking, dist, spd)
                )
                await ws.send_text(json.dumps({
                    'type': 'action_result',
                    'action': 'move',
                    'success': ok,
                    'message': msg,
                }, ensure_ascii=False))

            elif t == 'send_span':
                ang = float(cmd.get('angle_rad', 1.57))
                om = float(cmd.get('omega_radps', 0.5))
                loop = asyncio.get_running_loop()
                ok, msg = await loop.run_in_executor(
                    None, partial(self._send_span_blocking, ang, om)
                )
                await ws.send_text(json.dumps({
                    'type': 'action_result',
                    'action': 'span',
                    'success': ok,
                    'message': msg,
                }, ensure_ascii=False))

            elif t == 'start_calibrate':
                loop = asyncio.get_running_loop()
                ok, msg = await loop.run_in_executor(
                    None, self._send_calibrate_blocking
                )
                await ws.send_text(json.dumps({
                    'type': 'action_result',
                    'action': 'calibrate',
                    'success': ok,
                    'message': msg,
                }, ensure_ascii=False))

            elif t == 'cancel_actions':
                self._cancel_active_actions(source='cancel_actions')

            elif t == 'stop':
                if self._current_mode == 1:
                    self._cancel_active_actions(source='stop_move')
                else:
                    self._publish_twist(0.0, 0.0, log_source='stop')
                    self._cancel_active_actions(source='stop_twist')
                await ws.send_text(json.dumps({'type': 'stopped'}, ensure_ascii=False))

            elif t == 'reset_panel_log':
                ok, msg, deleted, path = self._reset_panel_log()
                await ws.send_text(json.dumps({
                    'type': 'panel_log_reset_result',
                    'success': ok,
                    'message': msg,
                    'deleted_count': deleted,
                    'path': path,
                }, ensure_ascii=False))

            elif t == 'ping':
                await ws.send_text(json.dumps({'type': 'pong', 't': time.time()}, ensure_ascii=False))

            else:
                await ws.send_text(json.dumps({'type': 'error', 'message': f'未知命令: {t}'}, ensure_ascii=False))
        except Exception as exc:
            self._plog('ws_cmd_error', cmd_type=t, error=str(exc))
            await ws.send_text(json.dumps({'type': 'error', 'message': str(exc)}, ensure_ascii=False))

    async def handle_ws_monitor_command(self, ws: WebSocket, cmd: dict[str, Any]) -> None:
        t = cmd.get('type')
        if t != 'ping':
            self._plog('ws_monitor_cmd', cmd_type=t, cmd=cmd)
        try:
            if t == 'set_monitor':
                mid = int(cmd.get('motor_id', 0))
                if mid not in self._monitor_slots:
                    await ws.send_text(json.dumps({
                        'type': 'error',
                        'message': f'未知 motor_id={mid}',
                    }, ensure_ascii=False))
                    return
                enabled = bool(cmd.get('enable', False))
                rate_hz = _clamp(float(cmd.get('rate_hz', 15.0)), 1.0, 30.0)
                self._monitor_slots[mid]['enabled'] = enabled
                self._monitor_slots[mid]['rate_hz'] = rate_hz
                if enabled:
                    self._monitor_slots[mid]['last_poll'] = 0.0
                await ws.send_text(json.dumps({
                    'type': 'monitor_ack',
                    'motor_id': mid,
                    'enabled': enabled,
                    'rate_hz': rate_hz,
                }, ensure_ascii=False))

            elif t == 'fetch_state':
                mid = int(cmd.get('motor_id', 0))
                loop = asyncio.get_running_loop()
                state = await loop.run_in_executor(
                    None, partial(self._get_state_blocking, mid)
                )
                self._plog(
                    'motor_state',
                    channel='fetch',
                    motor_id=mid,
                    success=bool(state.get('success')),
                    rpm=int(state.get('rpm', 0)),
                    position=int(state.get('position', 0)),
                    phase_current_ma=int(state.get('phase_current_ma', 0)),
                    stalled=bool(state.get('stalled', False)),
                    error_code=int(state.get('error_code', 0)),
                )
                await ws.send_text(json.dumps({
                    'type': 'motor_state',
                    'state': state,
                }, ensure_ascii=False))

            elif t == 'get_pid':
                mid = int(cmd.get('motor_id', 0))
                loop = asyncio.get_running_loop()
                ok, msg, p, i, d = await loop.run_in_executor(
                    None, partial(self._get_pid_blocking, mid)
                )
                await ws.send_text(json.dumps({
                    'type': 'pid_result',
                    'motor_id': mid,
                    'success': ok,
                    'message': msg,
                    'p': p,
                    'i': i,
                    'd': d,
                }, ensure_ascii=False))

            elif t == 'set_pid':
                mid = int(cmd.get('motor_id', 0))
                p = int(cmd.get('p', 0))
                i = int(cmd.get('i', 0))
                d = int(cmd.get('d', 0))
                loop = asyncio.get_running_loop()
                ok, msg, rp, ri, rd = await loop.run_in_executor(
                    None, partial(self._set_pid_blocking, mid, p, i, d)
                )
                await ws.send_text(json.dumps({
                    'type': 'pid_result',
                    'motor_id': mid,
                    'success': ok,
                    'message': msg,
                    'p': rp,
                    'i': ri,
                    'd': rd,
                }, ensure_ascii=False))

            elif t == 'set_chassis_pid':
                p = int(cmd.get('p', 0))
                i = int(cmd.get('i', 0))
                d = int(cmd.get('d', 0))
                self._plog('set_chassis_pid_request', p=p, i=i, d=d)
                loop = asyncio.get_running_loop()
                chassis_results: list[dict[str, Any]] = []
                for mid in (5, 4):
                    if mid == 4:
                        await asyncio.sleep(0.35)
                    ok, msg, rp, ri, rd = await loop.run_in_executor(
                        None, partial(self._set_pid_blocking, mid, p, i, d)
                    )
                    chassis_results.append({
                        'motor_id': mid,
                        'success': ok,
                        'message': msg,
                        'p': rp,
                        'i': ri,
                        'd': rd,
                    })
                    await ws.send_text(json.dumps({
                        'type': 'pid_result',
                        'motor_id': mid,
                        'success': ok,
                        'message': msg,
                        'p': rp,
                        'i': ri,
                        'd': rd,
                    }, ensure_ascii=False))
                self._plog(
                    'set_chassis_pid_result',
                    p=p,
                    i=i,
                    d=d,
                    all_ok=all(r['success'] for r in chassis_results),
                    results=chassis_results,
                )

            elif t == 'ping':
                await ws.send_text(json.dumps({'type': 'pong', 't': time.time()}, ensure_ascii=False))

            else:
                await ws.send_text(json.dumps({'type': 'error', 'message': f'未知命令: {t}'}, ensure_ascii=False))
        except Exception as exc:
            self._plog('ws_monitor_cmd_error', cmd_type=t, error=str(exc))
            await ws.send_text(json.dumps({'type': 'error', 'message': str(exc)}, ensure_ascii=False))


# ---------------- FastAPI app & HTML loader ----------------

_HTML_NO_CACHE_HEADERS = {
    'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
    'Pragma': 'no-cache',
}


def _load_newest_html(candidates: list[str]) -> str | None:
    """在多个候选路径中选 mtime 最新且存在的 HTML 文件（开发时优先于陈旧 install）。"""
    best_path: str | None = None
    best_mtime = -1.0
    seen: set[str] = set()
    for raw in candidates:
        try:
            p = os.path.realpath(raw)
        except OSError:
            continue
        if p in seen:
            continue
        seen.add(p)
        try:
            if not os.path.isfile(p):
                continue
            mtime = os.path.getmtime(p)
        except OSError:
            continue
        if mtime > best_mtime:
            best_mtime = mtime
            best_path = p
    if best_path is None:
        return None
    with open(best_path, 'r', encoding='utf-8') as f:
        return f.read()


def _load_index_html() -> str:
    """Load the control panel HTML. Falls back to a minimal page if file missing."""
    candidates: list[str] = []
    if HAS_AMENT:
        try:
            share = get_package_share_directory('manual_control')
            candidates.append(os.path.join(share, 'web', 'index.html'))
        except Exception:
            pass

    # Dev fallbacks (source tree)
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.extend([
        os.path.join(here, '..', '..', 'web', 'index.html'),  # from manual_control/manual_control/
        os.path.join(os.getcwd(), 'web', 'index.html'),
        os.path.join('/home/yuxuan/robot_ws/src/robot_control/manual_control', 'web', 'index.html'),
    ])

    loaded = _load_newest_html(candidates)
    if loaded is not None:
        return loaded

    # Minimal fallback UI (still functional for basic tests)
    return """<!doctype html>
<html><head><meta charset="utf-8"><title>Robot Panel (Fallback)</title>
<style>body{font-family:sans-serif;background:#111;color:#ddd;padding:2rem}</style></head>
<body>
<h1>Web Control Panel - Fallback</h1>
<p>index.html 未找到。请执行 colcon build 后重试，或检查 web/index.html 是否存在。</p>
<p>WS 地址: <code id="wsurl"></code></p>
<script>
const ws = new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws');
ws.onopen = () => document.getElementById('wsurl').textContent = 'connected';
ws.onmessage = e => console.log('WS:', e.data);
</script>
</body></html>"""


def _load_monitor_html() -> str:
    candidates: list[str] = []
    if HAS_AMENT:
        try:
            share = get_package_share_directory('manual_control')
            candidates.append(os.path.join(share, 'web', 'monitor.html'))
        except Exception:
            pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.extend([
        os.path.join(here, '..', '..', 'web', 'monitor.html'),
        os.path.join('/home/yuxuan/robot_ws/src/robot_control/manual_control', 'web', 'monitor.html'),
    ])
    loaded = _load_newest_html(candidates)
    if loaded is not None:
        return loaded
    return '<html><body><h1>monitor.html 未找到</h1><p><a href="/">返回控制页</a></p></body></html>'


def create_app(node: WebControlPanel) -> FastAPI:
    app = FastAPI(title="Robot Web Control Panel", version="1.0")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.get("/", response_class=HTMLResponse)
    async def get_index():
        html = _load_index_html()
        return HTMLResponse(content=html, status_code=200, headers=_HTML_NO_CACHE_HEADERS)

    @app.get("/monitor", response_class=HTMLResponse)
    async def get_monitor():
        html = _load_monitor_html()
        return HTMLResponse(content=html, status_code=200, headers=_HTML_NO_CACHE_HEADERS)

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        node._ws_clients.add(websocket)
        if node._loop is None:
            node._loop = asyncio.get_running_loop()

        # initial state push
        await websocket.send_text(json.dumps({
            'type': 'status',
            'mode': node._current_mode,
            'arm': {'z': node._arm_z, 'j1': node._arm_j1, 'j2': node._arm_j2},
            'action_busy': node._chassis_action_busy(),
        }, ensure_ascii=False))

        try:
            while True:
                raw = await websocket.receive_text()
                try:
                    cmd = json.loads(raw)
                except Exception:
                    await websocket.send_text(json.dumps({'type': 'error', 'message': 'JSON 解析失败'}, ensure_ascii=False))
                    continue
                await node.handle_ws_command(websocket, cmd)
        except WebSocketDisconnect:
            pass
        finally:
            node._ws_clients.discard(websocket)

    @app.websocket("/ws/monitor")
    async def websocket_monitor(websocket: WebSocket):
        await websocket.accept()
        node._ws_monitor_clients.add(websocket)
        if node._loop is None:
            node._loop = asyncio.get_running_loop()
        try:
            while True:
                raw = await websocket.receive_text()
                try:
                    cmd = json.loads(raw)
                except Exception:
                    await websocket.send_text(json.dumps({'type': 'error', 'message': 'JSON 解析失败'}, ensure_ascii=False))
                    continue
                await node.handle_ws_monitor_command(websocket, cmd)
        except WebSocketDisconnect:
            pass
        finally:
            node._ws_monitor_clients.discard(websocket)

    return app


# ---------------- Entry point ----------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WebControlPanel()
    app = create_app(node)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    host = node.web_host
    port = node.web_port

    try:
        uvicorn.run(
            app,
            host=host,
            port=port,
            log_level="info",
            reload=False,
            access_log=False,
        )
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('正在关闭 Web 控制面板...')
        node._plog('session_end')
        node._panel_log.close()
        node._monitor_run = False
        node._cancel_active_actions(source='shutdown')
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=3.0)


if __name__ == '__main__':
    main()
