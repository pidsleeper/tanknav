#!/usr/bin/env python3
"""Republish LiDAR + IMU with system clock via static offset.
Offset computed once from first message, then never changes.
Both topics share same offset -> relative timing preserved.
No early messages dropped. High queue depth for 200Hz IMU."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu


class LivoxStampFix(Node):
    def __init__(self):
        super().__init__("livox_stamp_fix")
        self._lidar_pub = self.create_publisher(PointCloud2, "/livox/lidar_sys", 10)
        self._imu_pub = self.create_publisher(Imu, "/livox/imu_sys", 10)

        # LiDAR: BEST_EFFORT (livox driver default). IMU: RELIABLE (default).
        lidar_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._lidar_sub = self.create_subscription(PointCloud2, "/livox/lidar", self.lidar_cb, lidar_qos)
        # High depth to handle 200Hz IMU without dropping
        self._imu_sub = self.create_subscription(Imu, "/livox/imu", self.imu_cb, 500)

        self._offset_ns = None

    def _to_ns(self, stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _apply_offset(self, stamp):
        ns = self._to_ns(stamp) + self._offset_ns
        stamp.sec = int(ns // 1_000_000_000)
        stamp.nanosec = int(ns % 1_000_000_000)

    def imu_cb(self, msg):
        if self._offset_ns is None:
            self._offset_ns = self.get_clock().now().nanoseconds - self._to_ns(msg.header.stamp)
            self.get_logger().info(f"Offset: {self._offset_ns / 1e9:.2f}s (from first IMU)")
        self._apply_offset(msg.header.stamp)
        self._imu_pub.publish(msg)

    def lidar_cb(self, msg):
        if self._offset_ns is None:
            # Rare: LiDAR arrives before any IMU
            self._offset_ns = self.get_clock().now().nanoseconds - self._to_ns(msg.header.stamp)
            self.get_logger().info(f"Offset: {self._offset_ns / 1e9:.2f}s (from first LiDAR)")
        self._apply_offset(msg.header.stamp)
        self._lidar_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(LivoxStampFix())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
