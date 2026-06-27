# FAST-LIVO2 + PGO 回环建图模式
# 建图TF链: map(pgo发)→odom(bridge转发)→base_link(静态TF)
# 组件: 静态TF + 底盘 + 激光雷达 + RealSense + FAST-LIVO2 + 点云变换 + bridge + PGO + RViz

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 配置文件路径
    fastlivo_config = PathJoinSubstitution([
        FindPackageShare("fast_livo"), "config", "avia.yaml"
    ])
    livo_overrides = {
        "local_map.half_map_size": 50,
        "dynamic_sync.dynamic_img_sync_en": False,
        "parameter_blackboard.cam_model": "Pinhole",
        "parameter_blackboard.cam_width": 1280,
        "parameter_blackboard.cam_height": 720,
        "parameter_blackboard.scale": 0.5,
        "parameter_blackboard.cam_fx": 920.0,
        "parameter_blackboard.cam_fy": 920.0,
        "parameter_blackboard.cam_cx": 640.0,
        "parameter_blackboard.cam_cy": 360.0,
        "parameter_blackboard.cam_d0": 0.0,
        "parameter_blackboard.cam_d1": 0.0,
        "parameter_blackboard.cam_d2": 0.0,
        "parameter_blackboard.cam_d3": 0.0,
    }
    pgo_config = PathJoinSubstitution([
        FindPackageShare("tank_bringup"), "config", "pgo_mid360_fastlivo.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),

        # 1. 静态 TF: base_link → mid360_link, base_link → camera_link
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tank_bringup"), "launch", "robot_fastlivo.launch.py"
            ])),
        ),

        # 2. 底盘驱动 (tank_base)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tank_bringup"), "launch", "base.launch.py"
            ])),
        ),

        # 3. 激光雷达驱动 (Mid360, livox_ros_driver2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tank_bringup"), "launch", "lidar.launch.py"
            ])),
            launch_arguments={
                "publish_freq": "10.0",
                "frame_id": "mid360_link",
            }.items(),
        ),

        # 4. RealSense D435i 相机 (仅 color, 不启用 depth)
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="camera",
            name="camera",
            output="screen",
            parameters=[{
                "enable_color": True,
                "enable_depth": False,
            }],
        ),

        # 5. FAST-LIVO2 建图
        TimerAction(
            period=4.0,
            actions=[Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="laserMapping",
                parameters=[fastlivo_config, livo_overrides],
                output="screen",
                respawn=True,
            )],
        ),

        # 6. 点云坐标系转换: /cloud_registered → /cloud_body (供 pgo 使用)
        Node(
            package="tank_bringup",
            executable="cloud_frame_transform.py",
            name="cloud_frame_transform",
            output="screen",
        ),

        # 7. FAST-LIVO2 → odom 桥接: 转发 /aft_mapped_to_init → /odom (供 pgo 使用)
        Node(
            package="tank_bringup",
            executable="fastlivo_nav_bridge.py",
            name="fastlivo_nav_bridge",
            output="screen",
            parameters=[{
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
            }],
        ),

        # 8. PGO 回环建图 (基于 /cloud_body + /odom 发布 map→odom TF + 回环检测)
        Node(
            package="pgo",
            executable="pgo_node",
            name="pgo",
            output="screen",
            parameters=[{"config_path": pgo_config}],
        ),

        # 9. RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("fast_livo"), "rviz_cfg", "fast_livo2.rviz"
            ])],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
