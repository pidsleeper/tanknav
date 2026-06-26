# FAST-LIVO2 导航模式 bringup
# TF 链: map→odom(localizer) → odom→base_link(bridge动态) → base_link→mid360_link/camera_link(静态)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, LogInfo,
                             OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def nav_setup(context, *args, **kwargs):
    """组件 10: Nav2 导航栈(延迟启动) + 先验地图自动重定位."""
    from ament_index_python.packages import get_package_share_directory
    import os

    actions = []
    map_yaml = LaunchConfiguration("map").perform(context)
    pcd_map = LaunchConfiguration("pcd_map").perform(context)
    delay = float(LaunchConfiguration("nav2_start_delay").perform(context))

    if not map_yaml:
        actions.append(LogInfo(msg="Nav2 未启动: 缺少 map 参数"))
        return actions

    nav2_share = get_package_share_directory("tank_nav2")
    params_file = os.path.join(nav2_share, "config", "nav2_params.yaml")

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("tank_nav2"), "launch", "nav2_core.launch.py"
            ])
        ),
        launch_arguments={
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
            "map": map_yaml,
        }.items(),
    )
    actions.append(TimerAction(period=delay, actions=[nav2_include]))

    # 自动重定位: localizer 启动后 5s 加载先验 PCD 地图
    if pcd_map:
        actions.append(
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2", "service", "call",
                            "/localizer/relocalize",
                            "interface/srv/Relocalize",
                            f"{{pcd_path: '{pcd_map}', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}}",
                        ],
                        output="screen",
                    )
                ],
            )
        )
        actions.append(
            Node(
                package="tank_bringup",
                executable="initialpose_bridge.py",
                name="initialpose_bridge",
                output="screen",
                parameters=[{"pcd_path": pcd_map}],
            )
        )

    return actions


def generate_launch_description():
    # 所有配置路径通过 FindPackageShare + PathJoinSubstitution 构造
    fastlivo_cfg = PathJoinSubstitution([
        FindPackageShare("tank_bringup"), "config", "fastlivo_mid360_nav.yaml"
    ])
    localizer_cfg = PathJoinSubstitution([
        FindPackageShare("tank_bringup"), "config", "localizer_mid360_fastlivo.yaml"
    ])
    pcs_cfg = PathJoinSubstitution([
        FindPackageShare("tank_bringup"), "config", "pointcloud_to_scan.yaml"
    ])

    return LaunchDescription([
        # ========== 启动参数 ==========
        DeclareLaunchArgument(
            "map", default_value="",
            description="Nav2 地图 YAML 路径 (必填)"
        ),
        DeclareLaunchArgument(
            "pcd_map", default_value="",
            description="localizer 先验 PCD 地图路径 (用于重定位)"
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="是否启动 RViz2 可视化"
        ),
        DeclareLaunchArgument(
            "nav2_start_delay", default_value="8.0",
            description="Nav2 延迟启动秒数 (等待 SLAM 收敛 + localizer 初始化)"
        ),

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

        # 3. Mid360 激光雷达驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tank_bringup"), "launch", "lidar.launch.py"
            ])),
            launch_arguments={
                "publish_freq": "10.0",
                "frame_id": "mid360_link",
            }.items(),
        ),

        # 4. D435i 深度相机驱动 (仅彩色图，供 FAST-LIVO2 VIO 使用)
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace="camera",
            output="screen",
            parameters=[{
                "enable_color": True,
                "enable_depth": False,
            }],
        ),

        # 5. FAST-LIVO2 SLAM (导航模式, fastlivo_mid360_nav.yaml)
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[fastlivo_cfg],
            output="screen",
            respawn=True,
        ),

        # 6. 点云坐标系转换: /cloud_registered(world) → /cloud_body(base_link)
        Node(
            package="tank_bringup",
            executable="cloud_frame_transform.py",
            name="cloud_frame_transform",
            output="screen",
        ),

        # 7. FAST-LIVO2 → Nav2 桥接 (动态发布 odom→base_link TF + /odom topic)
        #    注意: 这里不发 static_transform_publisher, TF 由 bridge 动态维护
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

        # 8. ICP 重定位 (发布 map→odom TF, 订阅 /cloud_body + /odom)
        #    使用 localizer_mid360_fastlivo.yaml (NOT localizer_mid360.yaml)
        Node(
            package="localizer",
            executable="localizer_node",
            name="localizer",
            output="screen",
            parameters=[localizer_cfg],
        ),

        # 9. 点云转激光扫描 (cloud_in=/cloud_body body系, scan=/scan)
        #    注意: cloud_in 指向 /cloud_body (body 系), 不是 /cloud_registered (world 系)
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[pcs_cfg],
            remappings=[
                ("cloud_in", "/cloud_body"),
                ("scan", "/scan"),
            ],
        ),

        # 10. Nav2 导航栈 (延迟启动 + 先验地图自动重定位)
        OpaqueFunction(function=nav_setup),

        # 11. RViz2 可视化 (可选)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_nav",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("tank_bringup"), "rviz", "nav2_default_view.rviz"
            ])],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
