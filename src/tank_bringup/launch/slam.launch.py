from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fastlio_config = LaunchConfiguration("fastlio_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fastlio_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_bringup"), "config", "fastlio_mid360.yaml"]
                ),
            ),
            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[{"config_path": fastlio_config}],
            ),
        ]
    )
