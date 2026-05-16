from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "livox_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"]
                ),
            ),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            DeclareLaunchArgument("frame_id", default_value="mid360_link"),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        "xfer_format": 1,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": ParameterValue(
                            LaunchConfiguration("publish_freq"), value_type=float
                        ),
                        "output_data_type": 0,
                        "frame_id": LaunchConfiguration("frame_id"),
                        "lvx_file_path": "/home/livox/livox_test.lvx",
                        "user_config_path": LaunchConfiguration("livox_config"),
                        "cmdline_input_bd_code": "livox0000000001",
                    }
                ],
            ),
        ]
    )
