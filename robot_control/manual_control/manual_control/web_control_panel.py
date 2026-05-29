"""Web control panel for robot using FastAPI + WebSocket.

Provides browser-based manual control for chassis (Twist) and arm (JointState),
plus mode switching service and blocking move/span actions from robot_platform.
"""

import asyncio
import json
import os
import threading
import time
from functools import partial
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# 受保护导入 robot_interfaces 的自定义接口（.srv / .action）。
# 如果失败，给出清晰的中文错误提示，指导用户正确构建。
try:
    from robot_interfaces.srv import SetDriveMode
    from robot_interfaces.action import ChassisMove, ChassisSpan
except Exception as _e:  # noqa: BLE001
    raise RuntimeError(
        "无法导入 robot_interfaces 的自定义服务/动作接口（SetDriveMode、ChassisMove、ChassisSpan）。\n"
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


class WebControlPanel(Node):
    """ROS node + web server bridge for manual teleop."""

    def __init__(self) -> None:
        super().__init__('web_control_panel')

        # Topics & joint config (match robot_platform defaults)
        self.declare_parameter('chassis_cmd_vel_topic', '/chassis/cmd_vel')
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

        # Load params
        self._chassis_topic = str(self.get_parameter('chassis_cmd_vel_topic').value)
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

        # Publishers
        self._pub_twist = self.create_publisher(Twist, self._chassis_topic, 10)
        self._pub_joint = self.create_publisher(JointState, self._arm_topic, 10)

        # Service & Action clients
        self._mode_client = self.create_client(SetDriveMode, '/chassis/set_mode')
        self._move_client = ActionClient(self, ChassisMove, '/chassis/move')
        self._span_client = ActionClient(self, ChassisSpan, '/chassis/span')

        # Runtime state
        self._current_mode: int = 0  # 0=TWIST, 1=MOVE
        self._last_twist: tuple[float, float] = (0.0, 0.0)
        self._last_twist_time: float = 0.0
        self._active_move_gh = None
        self._active_span_gh = None

        # WebSocket clients (set of WebSocket objects) + event loop for thread-safe push
        self._ws_clients: set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None

        # Watchdog + status timer
        self.create_timer(0.1, self._watchdog_and_status)

        self.get_logger().info(
            f'Web control panel ready. Topics: chassis={self._chassis_topic}, arm={self._arm_topic}. '
            f'Web UI at http://{self.web_host}:{self.web_port}'
        )
        self._publish_arm()  # initial position

    # ---------------- ROS helpers ----------------

    def set_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._pub_twist.publish(msg)
        self._last_twist = (linear_x, angular_z)
        self._last_twist_time = time.monotonic()
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
            self._publish_twist(0.0, 0.0)

        # periodic status push
        self._safe_broadcast({
            'type': 'status',
            'mode': self._current_mode,
            'arm': {'z': self._arm_z, 'j1': self._arm_j1, 'j2': self._arm_j2},
            'twist': {'linear_x': self._last_twist[0], 'angular_z': self._last_twist[1]},
        })

    # Mode service (blocking wrapper for run_in_executor)
    def _set_mode_blocking(self, mode: int) -> tuple[bool, str]:
        if not self._mode_client.wait_for_service(timeout_sec=3.0):
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
            return bool(res.success), res.message or ''
        return False, '模式切换超时'

    # Action helpers
    def _send_move_blocking(self, distance_m: float, speed_mps: float) -> tuple[bool, str]:
        if self._current_mode != 1:
            return False, '必须先切换到 MOVE 模式'
        if not self._move_client.wait_for_server(timeout_sec=3.0):
            return False, 'Action /chassis/move 服务器不可用'
        goal = ChassisMove.Goal()
        goal.distance_m = float(distance_m)
        goal.speed_mps = float(speed_mps)
        send_fut = self._move_client.send_goal_async(goal, feedback_callback=self._on_move_feedback)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=4.0)
        if not send_fut.done():
            return False, '发送 Move Goal 超时'
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return False, 'Move Goal 被拒绝（检查当前模式）'
        self._active_move_gh = gh
        result_fut = gh.get_result_async()
        result_fut.add_done_callback(self._on_move_result)
        return True, 'Move Goal 已接受，开始执行'

    def _on_move_feedback(self, feedback_msg: Any) -> None:
        fb = feedback_msg.feedback
        self._safe_broadcast({
            'type': 'move_feedback',
            'distance_remaining_m': float(fb.distance_remaining_m),
        })

    def _on_move_result(self, fut: Any) -> None:
        self._active_move_gh = None
        try:
            res = fut.result()
            self._safe_broadcast({
                'type': 'action_result',
                'action': 'move',
                'success': bool(res.result.success),
                'message': 'Move 完成' if res.result.success else 'Move 失败或被取消',
            })
        except Exception as e:
            self._safe_broadcast({'type': 'action_result', 'action': 'move', 'success': False, 'message': str(e)})

    def _send_span_blocking(self, angle_rad: float, omega_radps: float) -> tuple[bool, str]:
        if self._current_mode != 1:
            return False, '必须先切换到 MOVE 模式'
        if not self._span_client.wait_for_server(timeout_sec=3.0):
            return False, 'Action /chassis/span 服务器不可用'
        goal = ChassisSpan.Goal()
        goal.angle_rad = float(angle_rad)
        goal.omega_radps = float(omega_radps)
        send_fut = self._span_client.send_goal_async(goal, feedback_callback=self._on_span_feedback)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=4.0)
        if not send_fut.done():
            return False, '发送 Span Goal 超时'
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return False, 'Span Goal 被拒绝（检查当前模式）'
        self._active_span_gh = gh
        result_fut = gh.get_result_async()
        result_fut.add_done_callback(self._on_span_result)
        return True, 'Span Goal 已接受，开始执行'

    def _on_span_feedback(self, feedback_msg: Any) -> None:
        fb = feedback_msg.feedback
        self._safe_broadcast({
            'type': 'span_feedback',
            'angle_remaining_rad': float(fb.angle_remaining_rad),
        })

    def _on_span_result(self, fut: Any) -> None:
        self._active_span_gh = None
        try:
            res = fut.result()
            self._safe_broadcast({
                'type': 'action_result',
                'action': 'span',
                'success': bool(res.result.success),
                'message': 'Span 完成' if res.result.success else 'Span 失败或被取消',
            })
        except Exception as e:
            self._safe_broadcast({'type': 'action_result', 'action': 'span', 'success': False, 'message': str(e)})

    def _cancel_active_actions(self) -> None:
        for gh, name in [(self._active_move_gh, 'move'), (self._active_span_gh, 'span')]:
            if gh is not None:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                self._safe_broadcast({'type': 'action_result', 'action': name, 'success': False, 'message': '已取消'})
        self._active_move_gh = None
        self._active_span_gh = None

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

    # ---------------- WebSocket command handling ----------------

    async def handle_ws_command(self, ws: WebSocket, cmd: dict[str, Any]) -> None:
        t = cmd.get('type')
        try:
            if t == 'set_twist':
                lx = float(cmd.get('linear_x', 0.0))
                az = float(cmd.get('angular_z', 0.0))
                self._publish_twist(lx, az)

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
                loop = asyncio.get_running_loop()
                success, message = await loop.run_in_executor(
                    None, partial(self._set_mode_blocking, mode)
                )
                await ws.send_text(json.dumps({
                    'type': 'mode_result',
                    'success': success,
                    'message': message,
                    'mode': self._current_mode,
                }, ensure_ascii=False))

            elif t == 'send_move':
                dist = float(cmd.get('distance_m', 0.5))
                spd = float(cmd.get('speed_mps', 0.2))
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

            elif t == 'cancel_actions':
                self._cancel_active_actions()

            elif t == 'stop':
                self._publish_twist(0.0, 0.0)
                self._cancel_active_actions()
                await ws.send_text(json.dumps({'type': 'stopped'}, ensure_ascii=False))

            elif t == 'ping':
                await ws.send_text(json.dumps({'type': 'pong', 't': time.time()}, ensure_ascii=False))

            else:
                await ws.send_text(json.dumps({'type': 'error', 'message': f'未知命令: {t}'}, ensure_ascii=False))
        except Exception as exc:
            await ws.send_text(json.dumps({'type': 'error', 'message': str(exc)}, ensure_ascii=False))


# ---------------- FastAPI app & HTML loader ----------------

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

    for p in candidates:
        try:
            if os.path.isfile(p):
                with open(p, 'r', encoding='utf-8') as f:
                    return f.read()
        except Exception:
            continue

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
        return HTMLResponse(content=html, status_code=200)

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

    return app


# ---------------- Entry point ----------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WebControlPanel()
    app = create_app(node)

    executor = MultiThreadedExecutor(num_threads=2)
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
        node._cancel_active_actions()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=3.0)


if __name__ == '__main__':
    main()
