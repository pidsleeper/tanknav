from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    actions = []
    use_base_driver = LaunchConfiguration("use_base_driver").perform(context)
    config_file = LaunchConfiguration("base_config")
    if use_base_driver.lower() != "true":
        actions.append(LogInfo(msg="`tank_base` launch skipped."))
        return actions

    try:
        get_package_share_directory("tank_base")
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg=(
                    "`tank_base` package is not in the workspace yet. "
                    "Reserve this layer for chassis serial communication, cmd_vel -> serial, "
                    "and battery/status feedback."
                )
            )
        )
        return actions

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("tank_base"), "launch", "base_driver.launch.py"]
                )
            ),
            launch_arguments={"config_file": config_file}.items(),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_base_driver", default_value="true"),
            DeclareLaunchArgument(
                "base_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("tank_base"), "config", "serial.yaml"]
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
