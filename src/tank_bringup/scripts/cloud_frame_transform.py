#!/usr/bin/env python3
"""
cloud_frame_transform.py — 将 FAST-LIVO2 发布的 camera_init 系(world/odom)点云
/cloud_registered 变换到 body 系(base_link)点云 /cloud_body，供 localizer/pgo 的 ICP 消费。

变换公式: p_body = R_world_to_body * (p_world - t_body_in_world)
其中 odom 消息 (/aft_mapped_to_init) 给出 body 在世界系(camera_init)中的位姿。

依赖: rclpy, numpy, sensor_msgs, nav_msgs — 均为 ROS 2 Humble 标准包, 无需额外安装。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


# ─── 四元数工具函数 (纯 Python, 零外部依赖) ───────────────────────────────

def quaternion_conjugate(qw: float, qx: float, qy: float, qz: float):
    """计算四元数的共轭 (逆旋转)。"""
    return (qw, -qx, -qy, -qz)


def quaternion_rotate_vector(q, v):
    """用四元数 q 旋转一组 3D 向量 v (N×3 numpy array)。

    参数:
        q: (qw, qx, qy, qz) — 单位四元数
        v: N×3 ndarray — 待旋转向量

    返回:
        N×3 ndarray — 旋转后向量

    公式: v' = v + 2 * cross(q_vec, cross(q_vec, v) + qw * v)
    参考: https://gamedev.stackexchange.com/a/50545
    """
    qw, qx, qy, qz = q
    q_vec = np.array([qx, qy, qz], dtype=np.float64)
    v = v.astype(np.float64, copy=False)
    t = 2.0 * np.cross(q_vec, v)
    return (v + qw * t + np.cross(q_vec, t)).astype(np.float32, copy=False)


# ─── 主节点 ───────────────────────────────────────────────────────────────

class CloudFrameTransform(Node):
    """从 world 系(camera_init)点云生成 body 系(base_link)点云。"""

    def __init__(self):
        super().__init__('cloud_frame_transform')

        # ── 参数 ──
        self.declare_parameter('input_cloud_topic', '/cloud_registered')
        self.declare_parameter('input_odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('output_cloud_topic', '/cloud_body')
        self.declare_parameter('output_frame', 'base_link')

        input_cloud = self.get_parameter('input_cloud_topic').value
        input_odom = self.get_parameter('input_odom_topic').value
        output_cloud = self.get_parameter('output_cloud_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        # ── 状态 ──
        self.latest_position = None      # (px, py, pz) ndarray
        self.latest_orientation = None   # (qw, qx, qy, qz) tuple
        self.odom_received = False

        # ── 订阅 ──
        self.cloud_sub = self.create_subscription(
            PointCloud2, input_cloud, self.cloud_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, input_odom, self.odom_callback, 10)

        # ── 发布 ──
        self.cloud_pub = self.create_publisher(
            PointCloud2, output_cloud, 10)

        self.get_logger().info(
            f'cloud_frame_transform 已启动:\n'
            f'  点云:  {input_cloud} (camera_init) -> {output_cloud} ({self.output_frame})\n'
            f'  里程计: {input_odom}'
        )

    def odom_callback(self, msg: Odometry):
        """缓存最新的 odom 位姿 (body 在 camera_init 系中的位姿)。"""
        pose = msg.pose.pose
        self.latest_position = np.array([
            pose.position.x, pose.position.y, pose.position.z
        ], dtype=np.float64)
        self.latest_orientation = (
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z
        )
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('已收到首帧里程计数据, 开始变换点云')

    def cloud_callback(self, msg: PointCloud2):
        """点云回调: camera_init → body 系变换。"""
        if not self.odom_received:
            return

        n = msg.width
        if n == 0:
            return

        # ── 1. 解析原消息 data buffer → numpy ──
        # 假设: 每个点前 3 个 float32 字段依次为 x, y, z (PCL toROSMsg 标准行为)
        stride = msg.point_step // 4          # float32 个数/点
        raw = np.frombuffer(msg.data, dtype=np.float32)
        xyz_world = raw.reshape(-1, stride)[:, :3]   # 视图, N×3

        # ── 2. 坐标系变换: world → body ──
        # p_body = R_w→b * (p_world - t_b_in_w)
        # R_w→b = conjugate(q_b→w), odom 给出的位姿即 body→world 变换
        p_rel = xyz_world - self.latest_position.astype(np.float32)
        q_conj = quaternion_conjugate(*self.latest_orientation)
        xyz_body = quaternion_rotate_vector(q_conj, p_rel)

        # ── 3. 写回新 xyz, 构造输出消息 ──
        raw_out = raw.copy()                          # 全量拷贝 buffer
        raw_out.reshape(-1, stride)[:, :3] = xyz_body  # 替换前 3 列

        out = PointCloud2()
        out.header.stamp = msg.header.stamp           # 保留原时间戳
        out.header.frame_id = self.output_frame
        out.height = msg.height
        out.width = msg.width
        out.fields = list(msg.fields)                  # 浅拷贝字段列表
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = raw_out.tobytes()

        self.cloud_pub.publish(out)


# ─── 入口 ──────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CloudFrameTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
