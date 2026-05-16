from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fastlio_config = LaunchConfiguration("fastlio_config")
    localizer_config = LaunchConfiguration("localizer_config")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("tank_bringup"), "launch", "slam.launch.py"])
        ),
        launch_arguments={"fastlio_config": fastlio_config}.items(),
    )

    return LaunchDescription(
        [
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
            slam_launch,
            Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[{"config_path": localizer_config}],
            ),
        ]
    )
