#!/usr/bin/env python3
"""Republish /scan with fresh timestamps to fix LiDAR clock drift issue."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanStampFix(Node):
    def __init__(self):
        super().__init__("scan_stamp_fix")
        self._pub = self.create_publisher(LaserScan, "/scan_fresh", 10)
        self._sub = self.create_subscription(
            LaserScan, "/scan", self.cb, 10
        )

    def cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ScanStampFix())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
