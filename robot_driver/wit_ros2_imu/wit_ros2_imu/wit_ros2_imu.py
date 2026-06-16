import math
import serial
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

key = 0
buff = {}
angularVelocity = [0.0, 0.0, 0.0]
acceleration = [0.0, 0.0, 0.0]
angle_degree = [0.0, 0.0, 0.0]


def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def handle_serial_data(raw_data):
    global buff, key, angle_degree, acceleration, angularVelocity
    angle_flag = False
    buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        buff = {}
        return False
    if key < 11:
        return False

    data_buff = list(buff.values())
    if buff[1] == 0x51:
        if check_sum(data_buff[0:10], data_buff[10]):
            acceleration = [
                hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)
            ]
        else:
            print('0x51 Check failure')

    elif buff[1] == 0x52:
        if check_sum(data_buff[0:10], data_buff[10]):
            angularVelocity = [
                hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180
                for i in range(0, 3)
            ]
        else:
            print('0x52 Check failure')

    elif buff[1] == 0x53:
        if check_sum(data_buff[0:10], data_buff[10]):
            angle_degree = [
                hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)
            ]
            angle_flag = True
        else:
            print('0x53 Check failure')
    else:
        buff = {}
        key = 0
        return False

    buff = {}
    key = 0
    return angle_flag


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [float(qx), float(qy), float(qz), float(qw)]


class IMUDriverNode(Node):
    """WIT IMU driver.

    Serial parsing runs in a background thread; ROS publish runs on executor timer
    so we never call Publisher.publish() from a non-executor thread (rclpy unsafe).
    """

    def __init__(self):
        super().__init__('imu_driver_node')

        self.declare_parameter('port', '/dev/imu_usb')
        self.declare_parameter('baud', 9600)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'base_link'
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        self._sample_lock = threading.Lock()
        self._has_sample = False
        self._accel = [0.0, 0.0, 0.0]
        self._gyro = [0.0, 0.0, 0.0]
        self._angles_deg = [0.0, 0.0, 0.0]

        self._driver_stop = threading.Event()
        self.create_timer(0.01, self._publish_timer_cb)

        self.get_logger().info(f"IMU 驱动启动，端口: {self.port}, 波特率: {self.baud}")
        self._driver_thread = threading.Thread(target=self._driver_loop, daemon=True)
        self._driver_thread.start()

    def _mark_angle_sample_ready(self):
        with self._sample_lock:
            self._accel = [float(v) for v in acceleration]
            self._gyro = [float(v) for v in angularVelocity]
            self._angles_deg = [float(v) for v in angle_degree]
            self._has_sample = True

    def _publish_timer_cb(self):
        with self._sample_lock:
            if not self._has_sample:
                return
            accel = self._accel
            gyro = self._gyro
            angles_deg = self._angles_deg

        try:
            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_msg.linear_acceleration.x = accel[0]
            self.imu_msg.linear_acceleration.y = accel[1]
            self.imu_msg.linear_acceleration.z = accel[2]
            self.imu_msg.angular_velocity.x = gyro[0]
            self.imu_msg.angular_velocity.y = gyro[1]
            self.imu_msg.angular_velocity.z = gyro[2]

            angle_radian = [angles_deg[i] * math.pi / 180.0 for i in range(3)]
            qua = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
            self.imu_msg.orientation.x = qua[0]
            self.imu_msg.orientation.y = qua[1]
            self.imu_msg.orientation.z = qua[2]
            self.imu_msg.orientation.w = qua[3]

            self.imu_pub.publish(self.imu_msg)
        except Exception as exc:
            self.get_logger().error(f'IMU publish 失败: {exc}')

    def _driver_loop(self):
        while rclpy.ok() and not self._driver_stop.is_set():
            try:
                self._driver_session()
            except Exception as exc:
                self.get_logger().error(f'IMU 驱动线程异常: {exc}')
            if not rclpy.ok() or self._driver_stop.is_set():
                break
            self.get_logger().warn('IMU 驱动 1 秒后重连串口...')
            time.sleep(1.0)

        self.get_logger().error('IMU 驱动线程已退出')

    def _driver_session(self):
        try:
            wt_imu = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.5)
            if not wt_imu.isOpen():
                wt_imu.open()
            self.get_logger().info("\033[32m串口打开成功\033[0m")
        except Exception as exc:
            self.get_logger().error(f"无法打开串口 {self.port}: {exc}")
            time.sleep(1.0)
            return

        while rclpy.ok() and not self._driver_stop.is_set():
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as exc:
                self.get_logger().error(f"串口异常: {exc}")
                break

            if buff_count > 0:
                buff_data = wt_imu.read(buff_count)
                for i in range(buff_count):
                    if handle_serial_data(buff_data[i]):
                        self._mark_angle_sample_ready()

        try:
            wt_imu.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = IMUDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._driver_stop.set()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
