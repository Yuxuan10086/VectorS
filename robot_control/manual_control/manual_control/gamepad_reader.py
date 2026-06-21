"""Linux /dev/input/js* 手柄后台读取（主控本地，不依赖浏览器 Gamepad API）。"""

from __future__ import annotations

import os
import select
import struct
import threading
import time
from typing import Callable

JS_EVENT_FMT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
FULL_SCALE = 32767

# tool/gamepad_mapping.yaml · Xbox 360
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 4
AXIS_LEFT_PAD_X = 6
AXIS_LEFT_PAD_Y = 7

# right_pad_* → 面键 Y / X / B / A（button 编号见 gamepad_mapping.yaml）
FACE_BUTTON_ACTION: dict[int, str] = {
    3: 'Y',  # right_pad_up
    2: 'X',  # right_pad_left
    1: 'B',  # right_pad_right
    0: 'A',  # right_pad_down
}

DEADZONE = 0.12
PAD_THRESHOLD = 0.5


class GamepadReader:
    """在独立线程中读取 js 设备，维护轴状态并计算 twist。"""

    def __init__(
        self,
        device_path: str,
        *,
        on_connection_change: Callable[[bool, str], None] | None = None,
        on_face_button: Callable[[str], None] | None = None,
        reopen_interval_sec: float = 1.0,
    ) -> None:
        self.device_path = device_path
        self.on_connection_change = on_connection_change
        self.on_face_button = on_face_button
        self.reopen_interval_sec = reopen_interval_sec
        self._run = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._axis: dict[int, int] = {}
        self._buttons: dict[int, int] = {}
        self._enabled = False
        self._max_lin = 0.15
        self._max_ang = 0.6
        self._connected = False
        self._device_name = ''

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name='gamepad_reader')
        self._thread.start()

    def stop(self) -> None:
        self._run = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = bool(enabled)

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def set_speed_limits(self, max_lin: float, max_ang_rad: float) -> None:
        with self._lock:
            self._max_lin = max(0.0, float(max_lin))
            self._max_ang = max(0.0, float(max_ang_rad))

    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def device_name(self) -> str:
        with self._lock:
            return self._device_name

    @staticmethod
    def _apply_deadzone(v: float, deadzone: float = DEADZONE) -> float:
        a = abs(v)
        if a < deadzone:
            return 0.0
        sign = -1.0 if v < 0 else 1.0
        return sign * (a - deadzone) / (1.0 - deadzone)

    def compute_twist(self) -> tuple[float, float, float, float]:
        """返回 (linear_x, angular_z, stick_norm_x, stick_norm_y)。"""
        with self._lock:
            raw = dict(self._axis)
            max_lin = self._max_lin
            max_ang = self._max_ang

        stick_x = self._apply_deadzone(raw.get(AXIS_RIGHT_STICK_X, 0) / FULL_SCALE)
        stick_y = self._apply_deadzone(raw.get(AXIS_RIGHT_STICK_Y, 0) / FULL_SCALE)
        pad_x = raw.get(AXIS_LEFT_PAD_X, 0) / FULL_SCALE
        pad_y = raw.get(AXIS_LEFT_PAD_Y, 0) / FULL_SCALE

        lx = -stick_y * max_lin
        az = -stick_x * max_ang

        if pad_y < -PAD_THRESHOLD:
            lx = max_lin
        elif pad_y > PAD_THRESHOLD:
            lx = -max_lin
        if pad_x < -PAD_THRESHOLD:
            az = max_ang
        elif pad_x > PAD_THRESHOLD:
            az = -max_ang

        return lx, az, stick_x, stick_y

    def _set_connected(self, connected: bool, name: str = '') -> None:
        changed = False
        with self._lock:
            if self._connected != connected or (connected and name and self._device_name != name):
                changed = True
            self._connected = connected
            self._device_name = name if connected else ''
            if not connected:
                self._axis = {}
                self._buttons = {}
        if changed and self.on_connection_change:
            self.on_connection_change(connected, name)

    def _loop(self) -> None:
        while self._run:
            if not os.path.exists(self.device_path):
                self._set_connected(False)
                time.sleep(self.reopen_interval_sec)
                continue
            try:
                self._read_loop()
            except OSError:
                self._set_connected(False)
                time.sleep(self.reopen_interval_sec)

    def _read_device_name(self) -> str:
        base = os.path.basename(self.device_path)
        name_path = f'/sys/class/input/{base}/device/name'
        try:
            with open(name_path, encoding='utf-8') as f:
                return f.read().strip()
        except OSError:
            return self.device_path

    def _read_loop(self) -> None:
        with open(self.device_path, 'rb', buffering=0) as dev:
            self._set_connected(True, self._read_device_name())
            while self._run:
                readable, _, _ = select.select([dev], [], [], 0.05)
                if not readable:
                    continue
                raw = dev.read(JS_EVENT_SIZE)
                if len(raw) != JS_EVENT_SIZE:
                    continue
                _time_ms, value, ev_type, number = struct.unpack(JS_EVENT_FMT, raw)
                ev_clean = ev_type & ~JS_EVENT_INIT
                if ev_type & JS_EVENT_INIT:
                    with self._lock:
                        if ev_clean == JS_EVENT_AXIS:
                            self._axis[number] = int(value)
                        elif ev_clean == JS_EVENT_BUTTON:
                            self._buttons[number] = int(value)
                    continue
                if ev_clean == JS_EVENT_AXIS:
                    with self._lock:
                        self._axis[number] = int(value)
                elif ev_clean == JS_EVENT_BUTTON:
                    with self._lock:
                        prev = self._buttons.get(number, 0)
                        self._buttons[number] = int(value)
                    if value == 1 and prev == 0:
                        action = FACE_BUTTON_ACTION.get(number)
                        if action and self.on_face_button:
                            self.on_face_button(action)
