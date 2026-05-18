from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fastlio_config = LaunchConfiguration("fastlio_config")
    pgo_config = LaunchConfiguration("pgo_config")
    use_rviz = LaunchConfiguration("use_rviz")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("tank_bringup"), "launch", "slam.launch.py"])
        ),
        launch_arguments={"fastlio_config": fastlio_config}.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_mapping",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("tank_bringup"), "rviz", "mapping.rviz"])],
        condition=IfCondition(use_rviz),
        output="screen",
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
                "pgo_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_bringup"), "config", "pgo_mid360.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            slam_launch,
            Node(
                package="pgo",
                namespace="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                parameters=[{"config_path": pgo_config}],
            ),
            rviz_node,
        ]
    )
