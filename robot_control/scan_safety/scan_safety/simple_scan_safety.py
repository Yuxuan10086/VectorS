"""Subscribe cmd_vel input + LaserScan; publish filtered cmd_vel when obstacle in forward sector."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def _angle_wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _shortest_angle_diff(a: float, b: float) -> float:
    return abs(_angle_wrap_pi(a - b))


class SimpleScanSafety(Node):
    def __init__(self) -> None:
        super().__init__('simple_scan_safety')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_in', 'cmd_vel_raw')
        self.declare_parameter('cmd_vel_out', 'cmd_vel')
        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_enabled_state', True)
        self.declare_parameter('enabled_topic', 'avoidance_enabled')

        self.declare_parameter('min_distance_m', 0.35)
        self.declare_parameter('forward_angle_rad', 0.0)
        self.declare_parameter('forward_half_angle_rad', math.radians(40.0))

        self.declare_parameter('allow_rotation_when_blocked', True)
        # True：仅禁止前进，允许倒车；False：有障碍时线速度整体清零
        self.declare_parameter('allow_reverse_when_blocked', False)

        self.declare_parameter('require_scan_before_move', True)

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        cmd_in = self.get_parameter('cmd_vel_in').get_parameter_value().string_value
        cmd_out = self.get_parameter('cmd_vel_out').get_parameter_value().string_value

        self._latest_scan = None  # type: LaserScan | None
        self._got_scan = False

        self._sub_scan = self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self._sub_cmd = self.create_subscription(Twist, cmd_in, self._on_cmd, 10)
        self._pub_cmd = self.create_publisher(Twist, cmd_out, 10)

        self._pub_enabled = None
        if self.get_parameter('publish_enabled_state').get_parameter_value().bool_value:
            etopic = self.get_parameter('enabled_topic').get_parameter_value().string_value
            self._pub_enabled = self.create_publisher(Bool, etopic, 10)

        self._timer = self.create_timer(0.05, self._tick_publish_enabled)

        self.get_logger().info(
            f'SimpleScanSafety: in={cmd_in} out={cmd_out} scan={scan_topic}'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._got_scan = True

    def _forward_clear(self, scan: LaserScan) -> bool:
        center = float(self.get_parameter('forward_angle_rad').value)
        half_w = float(self.get_parameter('forward_half_angle_rad').value)
        min_d = float(self.get_parameter('min_distance_m').value)
        rmin = scan.range_min
        rmax = scan.range_max

        best = math.inf
        n = len(scan.ranges)
        if n == 0:
            return True

        inc = scan.angle_increment
        if abs(inc) < 1e-9:
            return True

        for i in range(n):
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r):
                continue
            if r < rmin or r > rmax:
                continue
            ang = scan.angle_min + float(i) * inc
            if _shortest_angle_diff(ang, center) <= half_w:
                best = min(best, r)

        if best is math.inf:
            return True
        return best >= min_d

    def _apply_block(self, twist_in: Twist, blocked: bool) -> Twist:
        out = Twist()
        out.linear = twist_in.linear
        out.angular = twist_in.angular

        if not blocked:
            return out

        allow_rev = self.get_parameter('allow_reverse_when_blocked').get_parameter_value().bool_value
        allow_rot = self.get_parameter('allow_rotation_when_blocked').get_parameter_value().bool_value

        if allow_rev:
            if twist_in.linear.x > 0.0:
                out.linear.x = 0.0
        else:
            out.linear.x = 0.0

        out.linear.y = 0.0

        if not allow_rot:
            out.angular.x = 0.0
            out.angular.y = 0.0
            out.angular.z = 0.0

        return out

    def _on_cmd(self, msg: Twist) -> None:
        enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        req_scan = self.get_parameter('require_scan_before_move').get_parameter_value().bool_value

        if not enabled:
            self._pub_cmd.publish(msg)
            return

        if req_scan and not self._got_scan:
            safe = Twist()
            self._pub_cmd.publish(safe)
            return

        scan = self._latest_scan
        if scan is None:
            safe = Twist()
            self._pub_cmd.publish(safe)
            return

        forward_ok = self._forward_clear(scan)
        out = self._apply_block(msg, not forward_ok)
        self._pub_cmd.publish(out)

    def _tick_publish_enabled(self) -> None:
        if self._pub_enabled is None:
            return
        b = Bool()
        b.data = self.get_parameter('enabled').get_parameter_value().bool_value
        self._pub_enabled.publish(b)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleScanSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
