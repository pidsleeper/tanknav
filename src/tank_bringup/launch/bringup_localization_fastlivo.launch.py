from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fastlivo_config = PathJoinSubstitution([
        FindPackageShare("fast_livo"), "config", "avia.yaml"
    ])
    livo_overrides = {
        "local_map.map_sliding_en": True,
        "local_map.half_map_size": 50,
        "pcd_save.pcd_save_en": False,
        "publish.dense_map_en": False,
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
    localizer_config = PathJoinSubstitution([
        FindPackageShare("tank_bringup"), "config", "localizer_mid360_fastlivo.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz2",
        ),
        DeclareLaunchArgument(
            "pcd_map",
            default_value="",
            description="Path to PCD map for localizer relocalization",
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

        # 3. 激光雷达驱动 (livox_ros_driver2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tank_bringup"), "launch", "lidar.launch.py"
            ])),
            launch_arguments={
                "publish_freq": "10.0",
                "frame_id": "mid360_link",
            }.items(),
        ),

        # 4. RealSense D435i 相机 (仅彩色图像)
        Node(
            package="realsense2_camera",
            namespace="camera",
            executable="realsense2_camera_node",
            name="realsense2_camera_node",
            output="screen",
            parameters=[{
                "enable_color": True,
                "enable_depth": False,
            }],
        ),

        # 5. FAST-LIVO2 SLAM (导航配置: map_sliding_en=true)
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[fastlivo_config, livo_overrides],
            output="screen",
            respawn=True,
        ),

        # 6. 点云坐标系变换 (camera_init → body)
        Node(
            package="tank_bringup",
            executable="cloud_frame_transform.py",
            name="cloud_frame_transform",
            output="screen",
        ),

        # 7. FAST-LIVO2 桥接 (发布 /odom 与 odom→base_link TF)
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

        # 8. localizer ICP 重定位 (发布 map→odom TF)
        Node(
            package="localizer",
            namespace="localizer",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            parameters=[{"config_path": localizer_config}],
        ),

        # 9. RViz2 可视化 (可选)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_localization",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("tank_bringup"), "rviz", "navigation.rviz"
            ])],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
