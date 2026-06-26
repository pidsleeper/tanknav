# 机器人 URDF / 静态 TF 定义
#
# TF 树结构 (预期):
#   base_link
#     ├── mid360_link           ← 由本文件发布 (LiDAR 安装外参)
#     └── camera_link           ← 由本文件发布 (D435i 安装外参)
#           ├── camera_color_frame         ← realsense2_camera 自管
#           ├── camera_color_optical_frame ← realsense2_camera 自管
#           ├── camera_depth_frame         ← realsense2_camera 自管
#           ├── camera_depth_optical_frame ← realsense2_camera 自管
#           └── ... (imu/infra 等)
#
# 注意:
#   - camera_link 是 realsense2_camera TF 树的根，realsense 内部发布其子 frame
#   - 启动 realsense2_camera 时须设 base_frame_id=camera_link
#   - FAST-LIVO2 img_topic=/camera/camera/color/image_raw, frame 由 realsense 管理

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── LiDAR 外参 (mid360_link) ──
        DeclareLaunchArgument("mid360_x", default_value="0.0"),
        DeclareLaunchArgument("mid360_y", default_value="0.0"),
        DeclareLaunchArgument("mid360_z", default_value="0.0"),
        DeclareLaunchArgument("mid360_yaw", default_value="0.0"),
        DeclareLaunchArgument("mid360_pitch", default_value="0.0"),
        DeclareLaunchArgument("mid360_roll", default_value="0.0"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("mid360_frame", default_value="mid360_link"),

        # ── 相机外参 (camera_link，即 D435i 安装位姿) ──
        DeclareLaunchArgument("cam_x", default_value="0.0"),
        DeclareLaunchArgument("cam_y", default_value="0.0"),
        DeclareLaunchArgument("cam_z", default_value="0.1"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_frame", default_value="camera_link"),

        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_to_mid360_tf",
            arguments=[
                LaunchConfiguration("mid360_x"), LaunchConfiguration("mid360_y"),
                LaunchConfiguration("mid360_z"), LaunchConfiguration("mid360_yaw"),
                LaunchConfiguration("mid360_pitch"), LaunchConfiguration("mid360_roll"),
                LaunchConfiguration("base_frame"), LaunchConfiguration("mid360_frame"),
            ],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_to_camera_tf",
            arguments=[
                LaunchConfiguration("cam_x"), LaunchConfiguration("cam_y"),
                LaunchConfiguration("cam_z"), LaunchConfiguration("cam_yaw"),
                LaunchConfiguration("cam_pitch"), LaunchConfiguration("cam_roll"),
                LaunchConfiguration("base_frame"), LaunchConfiguration("cam_frame"),
            ],
        ),
    ])
