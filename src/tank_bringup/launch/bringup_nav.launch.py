from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_base"), "config", "serial.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "livox_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"]
                ),
            ),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            DeclareLaunchArgument("frame_id", default_value="mid360_link"),
            DeclareLaunchArgument(
                "fastlio_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_bringup"), "config", "fastlio_mid360.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "localizer_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_bringup"), "config", "localizer_mid360.yaml"]
                ),
            ),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "pcd_map",
                default_value="",
                description="Path to PCD map for localizer relocalization",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_nav2"), "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("nav2_start_delay", default_value="8.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("tank_bringup"), "launch", "robot.launch.py"])
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("tank_bringup"), "launch", "base.launch.py"])
                ),
                launch_arguments={"base_config": LaunchConfiguration("base_config")}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("tank_bringup"), "launch", "lidar.launch.py"])
                ),
                launch_arguments={
                    "livox_config": LaunchConfiguration("livox_config"),
                    "publish_freq": LaunchConfiguration("publish_freq"),
                    "frame_id": LaunchConfiguration("frame_id"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tank_bringup"), "launch", "localization.launch.py"]
                    )
                ),
                launch_arguments={
                    "fastlio_config": LaunchConfiguration("fastlio_config"),
                    "localizer_config": LaunchConfiguration("localizer_config"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tank_bringup"), "launch", "navigation.launch.py"]
                    )
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "pcd_map": LaunchConfiguration("pcd_map"),
                    "nav2_params": LaunchConfiguration("nav2_params"),
                    "nav2_start_delay": LaunchConfiguration("nav2_start_delay"),
                }.items(),
            ),
        ]
    )
