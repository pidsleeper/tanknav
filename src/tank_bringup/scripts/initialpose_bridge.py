#!/usr/bin/env python3
"""Bridge /initialpose -> /localizer/relocalize service call.
Allows RViz 2D Pose Estimate to also set the localizer's initial position."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from interface.srv import Relocalize

import math
import threading


class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__("initialpose_bridge")
        self._lock = threading.Lock()
        self._pcd_path = self.declare_parameter("pcd_path", "").value
        self._reloc_client = self.create_client(Relocalize, "/localizer/relocalize")
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_cb, 10
        )
        self.get_logger().info("InitialPoseBridge started")

    def pose_cb(self, msg):
        if not self._pcd_path:
            return
        if not self._reloc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/localizer/relocalize not available")
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        req = Relocalize.Request()
        req.pcd_path = self._pcd_path
        req.x = p.x
        req.y = p.y
        req.z = p.z
        req.yaw = yaw
        req.pitch = 0.0
        req.roll = 0.0

        with self._lock:
            future = self._reloc_client.call_async(req)
            future.add_done_callback(self.done_cb)

    def done_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Relocalized: {resp.message}")
            else:
                self.get_logger().warn(f"Relocalize failed: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Relocalize error: {e}")


def main():
    rclpy.init()
    node = InitialPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
