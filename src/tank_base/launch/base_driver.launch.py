from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_base"), "config", "serial.yaml"]
                ),
            ),
            Node(
                package="tank_base",
                executable="chassis_driver_node",
                name="chassis_driver_node",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
