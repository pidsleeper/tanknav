import os
import select
import termios
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool


FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D
TX_FRAME_SIZE = 11
RX_FRAME_SIZE = 24
ACC_SCALE = 1672.0
GYRO_SCALE = 3753.0


def bcc_checksum(payload: bytes) -> int:
    checksum = 0
    for value in payload:
        checksum ^= value
    return checksum


def to_int16(high: int, low: int) -> int:
    value = (high << 8) | low
    if value & 0x8000:
        value -= 0x10000
    return value


def clamp_int16(value: int) -> int:
    return max(-32768, min(32767, value))


class ChassisSerial:
    def __init__(self, port: str, baudrate: int) -> None:
        self._port = port
        self._baudrate = baudrate
        self._fd: Optional[int] = None

    def open(self) -> None:
        baud_attr = {
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
            230400: termios.B230400,
            460800: termios.B460800,
            921600: termios.B921600,
        }.get(self._baudrate)
        if baud_attr is None:
            raise ValueError(f"Unsupported baudrate: {self._baudrate}")

        fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = baud_attr
        attrs[5] = baud_attr
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        self._fd = fd

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def read_available(self) -> bytes:
        if self._fd is None:
            return b""
        ready, _, _ = select.select([self._fd], [], [], 0.0)
        if not ready:
            return b""
        try:
            return os.read(self._fd, 4096)
        except BlockingIOError:
            return b""

    def write_frame(self, frame: bytes) -> None:
        if self._fd is None:
            return
        os.write(self._fd, frame)


class ChassisDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("chassis_driver_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "base_feedback")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("feedback_odom_topic", "/base/feedback_odom")
        self.declare_parameter("imu_topic", "/base/raw_imu")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("motor_state_topic", "/base/motor_enabled")
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("feedback_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_feedback_odom", True)
        self.declare_parameter("publish_feedback_tf", False)

        self._port = self.get_parameter("port").value
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._base_frame = self.get_parameter("base_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._publish_imu = bool(self.get_parameter("publish_imu").value)
        self._publish_feedback_odom = bool(
            self.get_parameter("publish_feedback_odom").value
        )

        self._latest_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._rx_buffer = bytearray()

        self._serial = ChassisSerial(self._port, self._baudrate)
        try:
            self._serial.open()
        except OSError as exc:
            self.get_logger().error(
                f"Failed to open chassis serial {self._port}: {exc}"
            )
            raise
        self.get_logger().info(
            f"Opened chassis serial on {self._port} @ {self._baudrate} baud"
        )

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        feedback_odom_topic = self.get_parameter("feedback_odom_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        battery_topic = self.get_parameter("battery_topic").value
        motor_state_topic = self.get_parameter("motor_state_topic").value

        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 20)
        self._odom_pub = self.create_publisher(Odometry, feedback_odom_topic, 20)
        self._imu_pub = self.create_publisher(Imu, imu_topic, 20)
        self._battery_pub = self.create_publisher(BatteryState, battery_topic, 20)
        self._motor_state_pub = self.create_publisher(Bool, motor_state_topic, 20)

        cmd_period = max(0.01, 1.0 / float(self.get_parameter("cmd_rate_hz").value))
        poll_period = max(
            0.005, 1.0 / float(self.get_parameter("feedback_rate_hz").value)
        )

        self.create_timer(cmd_period, self._send_latest_cmd)
        self.create_timer(poll_period, self._poll_serial)

    def destroy_node(self):
        try:
            self._serial.close()
        finally:
            super().destroy_node()

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _send_latest_cmd(self) -> None:
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        cmd = Twist()
        if age <= self._cmd_timeout:
            cmd = self._latest_cmd

        vx_mm_s = clamp_int16(int(round(cmd.linear.x * 1000.0)))
        vy_mm_s = clamp_int16(int(round(cmd.linear.y * 1000.0)))
        wz_mrad_s = clamp_int16(int(round(cmd.angular.z * 1000.0)))

        payload = bytearray(
            [
                FRAME_HEAD,
                0x00,
                0x00,
                (vx_mm_s >> 8) & 0xFF,
                vx_mm_s & 0xFF,
                (vy_mm_s >> 8) & 0xFF,
                vy_mm_s & 0xFF,
                (wz_mrad_s >> 8) & 0xFF,
                wz_mrad_s & 0xFF,
            ]
        )
        payload.append(bcc_checksum(payload))
        payload.append(FRAME_TAIL)
        self._serial.write_frame(bytes(payload))

    def _poll_serial(self) -> None:
        self._rx_buffer.extend(self._serial.read_available())
        while True:
            start = self._rx_buffer.find(bytes([FRAME_HEAD]))
            if start < 0:
                self._rx_buffer.clear()
                return
            if start > 0:
                del self._rx_buffer[:start]
            if len(self._rx_buffer) < RX_FRAME_SIZE:
                return
            frame = bytes(self._rx_buffer[:RX_FRAME_SIZE])
            del self._rx_buffer[:RX_FRAME_SIZE]
            if frame[-1] != FRAME_TAIL:
                continue
            if bcc_checksum(frame[:22]) != frame[22]:
                self.get_logger().warn("Dropped chassis frame with invalid checksum")
                continue
            self._publish_feedback(frame)

    def _publish_feedback(self, frame: bytes) -> None:
        motor_enabled = frame[1] == 0x00

        vx = to_int16(frame[2], frame[3]) / 1000.0
        vy = to_int16(frame[4], frame[5]) / 1000.0
        wz = to_int16(frame[6], frame[7]) / 1000.0

        ax = to_int16(frame[8], frame[9]) / ACC_SCALE
        ay = to_int16(frame[10], frame[11]) / ACC_SCALE
        az = to_int16(frame[12], frame[13]) / ACC_SCALE

        gx = to_int16(frame[14], frame[15]) / GYRO_SCALE
        gy = to_int16(frame[16], frame[17]) / GYRO_SCALE
        gz = to_int16(frame[18], frame[19]) / GYRO_SCALE

        voltage = to_int16(frame[20], frame[21]) / 1000.0
        stamp = self.get_clock().now().to_msg()

        motor_msg = Bool()
        motor_msg.data = motor_enabled
        self._motor_state_pub.publish(motor_msg)

        battery_msg = BatteryState()
        battery_msg.header.stamp = stamp
        battery_msg.voltage = float(voltage)
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.present = True
        self._battery_pub.publish(battery_msg)

        if self._publish_feedback_odom:
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = self._odom_frame
            odom_msg.child_frame_id = self._base_frame
            odom_msg.twist.twist.linear.x = float(vx)
            odom_msg.twist.twist.linear.y = float(vy)
            odom_msg.twist.twist.angular.z = float(wz)
            self._odom_pub.publish(odom_msg)

        if self._publish_imu:
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self._base_frame
            imu_msg.linear_acceleration.x = float(ax)
            imu_msg.linear_acceleration.y = float(ay)
            imu_msg.linear_acceleration.z = float(az)
            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)
            imu_msg.orientation_covariance[0] = -1.0
            self._imu_pub.publish(imu_msg)


def main() -> None:
    rclpy.init()
    node = ChassisDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
