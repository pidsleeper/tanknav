#!/usr/bin/env python3
"""
FAST-LIVO2 → Nav2 桥接节点

将 FAST-LIVO2 的定位输出转换为 Nav2 标准接口:
   - /aft_mapped_to_init (Odometry) → /odom + odom→base_link TF
   - camera_init ↔ odom（里程计原点），aft_mapped ↔ base_link（车体）
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf2_ros


class FastlivoNavBridge(Node):
    def __init__(self):
        super().__init__("fastlivo_nav_bridge")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("slam_odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("nav_odom_topic", "/odom")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_odom", True)

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # TF broadcaster: odom → base_link (from SLAM pose)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 订阅 FAST-LIVO2 里程计
        slam_topic = self.get_parameter("slam_odom_topic").value
        self.slam_odom_sub = self.create_subscription(
            Odometry, slam_topic, self._slam_odom_cb, 10
        )

        # 发布 Nav2 标准里程计
        nav_topic = self.get_parameter("nav_odom_topic").value
        self.nav_odom_pub = self.create_publisher(Odometry, nav_topic, 10)

        self.get_logger().info(
            f"FAST-LIVO2→Nav2 bridge: {slam_topic} → {nav_topic}, "
            f"TF: {self.odom_frame}→{self.base_frame}"
        )

    def _slam_odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp
        pose = msg.pose.pose
        twist = msg.twist.twist

        # 发布 odom → base_link TF (FAST-LIVO2 camera_init → base_link)
        if self.get_parameter("publish_tf").value:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = pose.position.x
            tf_msg.transform.translation.y = pose.position.y
            tf_msg.transform.translation.z = pose.position.z
            tf_msg.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        # 转发里程计（保持原始信息，只改 frame_id）
        if self.get_parameter("publish_odom").value:
            nav_odom = Odometry()
            nav_odom.header.stamp = stamp
            nav_odom.header.frame_id = self.odom_frame
            nav_odom.child_frame_id = self.base_frame
            nav_odom.pose.pose = pose
            nav_odom.twist.twist = twist
            self.nav_odom_pub.publish(nav_odom)


def main():
    rclpy.init()
    node = FastlivoNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
