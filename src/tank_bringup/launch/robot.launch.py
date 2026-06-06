from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mid360_x", default_value="0.0"),
            DeclareLaunchArgument("mid360_y", default_value="0.0"),
            DeclareLaunchArgument("mid360_z", default_value="0.0"),
            DeclareLaunchArgument("mid360_yaw", default_value="0.0"),
            # Mid360 向下倾斜 25°（俯视角），负值 = 俯视。
            # 若雷达水平安装，改回 default_value="0.0"。
            DeclareLaunchArgument("mid360_pitch", default_value="-0.43633"),
            DeclareLaunchArgument("mid360_roll", default_value="0.0"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("mid360_frame", default_value="mid360_link"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_mid360_tf",
                arguments=[
                    LaunchConfiguration("mid360_x"),
                    LaunchConfiguration("mid360_y"),
                    LaunchConfiguration("mid360_z"),
                    LaunchConfiguration("mid360_yaw"),
                    LaunchConfiguration("mid360_pitch"),
                    LaunchConfiguration("mid360_roll"),
                    LaunchConfiguration("base_frame"),
                    LaunchConfiguration("mid360_frame"),
                ],
                output="screen",
            ),
        ]
    )
