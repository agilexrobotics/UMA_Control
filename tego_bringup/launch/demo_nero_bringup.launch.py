"""Bring up the Nero example arms, teleoperation, and tego control bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    left_arm_port = LaunchConfiguration("left_arm_port")
    right_arm_port = LaunchConfiguration("right_arm_port")
    left_port_name = LaunchConfiguration("left_port_name")
    right_port_name = LaunchConfiguration("right_port_name")
    log_level = LaunchConfiguration("log_level")
    start_arm = LaunchConfiguration("start_arm")
    start_teleop = LaunchConfiguration("start_teleop")

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("agx_arm_ctrl"), "launch", "start_double_nero.launch.py"]
            )
        ),
        launch_arguments={
            "left_can_port": left_arm_port,
            "right_can_port": right_arm_port,
            "log_level": log_level,
        }.items(),
        condition=IfCondition(start_arm),
    )
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("agx_arm_ctrl"), "launch", "teleop_double_nero.launch.py"]
            )
        ),
        condition=IfCondition(start_teleop),
    )
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tego_bringup"), "launch", "double_nero_remote.launch.py"]
            )
        ),
        launch_arguments={
            "left_port_name": left_port_name,
            "right_port_name": right_port_name,
            "gripper_width_min": LaunchConfiguration("gripper_width_min"),
            "gripper_width_max": LaunchConfiguration("gripper_width_max"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "log_level": log_level,
        }.items(),
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument("left_arm_port", default_value="left_nero"),
            DeclareLaunchArgument("right_arm_port", default_value="right_nero"),
            DeclareLaunchArgument("left_port_name", default_value="left_tego"),
            DeclareLaunchArgument("right_port_name", default_value="right_tego"),
            DeclareLaunchArgument("gripper_width_min", default_value="0.00"),
            DeclareLaunchArgument("gripper_width_max", default_value="0.07"),
            DeclareLaunchArgument("pub_rate", default_value="200.0"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("start_arm", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("start_teleop", default_value="true", choices=["true", "false"]),
            arm_launch,
            teleop_launch,
            bridge_launch,
        ]
    )
