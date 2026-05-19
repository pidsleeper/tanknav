from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    actions = []
    map_yaml = LaunchConfiguration("map").perform(context)
    pcd_map = LaunchConfiguration("pcd_map").perform(context)
    params_file = LaunchConfiguration("nav2_params").perform(context)
    nav2_start_delay = float(LaunchConfiguration("nav2_start_delay").perform(context))

    if not map_yaml:
        actions.append(LogInfo(msg="Nav2 bringup skipped because `map` was not provided."))
        return actions

    if not params_file:
        actions.append(LogInfo(msg="Nav2 bringup skipped because `nav2_params` was not provided."))
        return actions

    actions.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare("tank_bringup"), "config", "pointcloud_to_scan.yaml"]
                )
            ],
            remappings=[
                ("cloud_in", "/fastlio2/body_cloud"),
                ("scan", "/scan"),
            ],
        )
    )

    actions.append(
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_yaml}],
        )
    )

    actions.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "node_names": ["map_server"],
                }
            ],
        )
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tank_nav2"), "launch", "nav2_core.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
            "use_sim_time": "false",
        }.items(),
    )

    actions.append(TimerAction(period=nav2_start_delay, actions=[nav2_navigation]))

    # Auto-relocalize: load PCD map into localizer 5s after startup
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "pcd_map",
                default_value="",
                description="Path to PCD map for localizer relocalization (auto-loaded)",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_nav2"), "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("nav2_start_delay", default_value="8.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
