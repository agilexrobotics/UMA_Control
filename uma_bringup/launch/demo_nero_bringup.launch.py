"""Bring up the Nero example arms, teleoperation, and uma control bridges.

One entry for both backends: ``backend:=cpp/python``(默认 cpp)。
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from agx_arm_ctrl.robot_profile import load_profile
from uma_bringup.bridge_launch import bridge_wrapper_name, recording_control_node


def generate_launch_description() -> LaunchDescription:
    left_arm_port = LaunchConfiguration("left_arm_port")
    right_arm_port = LaunchConfiguration("right_arm_port")
    left_port_name = LaunchConfiguration("left_port_name")
    right_port_name = LaunchConfiguration("right_port_name")
    log_level = LaunchConfiguration("log_level")
    start_arm = LaunchConfiguration("start_arm")
    start_teleop = LaunchConfiguration("start_teleop")
    profile_path = LaunchConfiguration("profile_path")
    backend = LaunchConfiguration("backend")

    def build_arm(context):
        profile = load_profile(
            str(Path(profile_path.perform(context))), validate_robot=False
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("agx_arm_ctrl"),
                            "launch",
                            "start_double_arm.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "left_can_port": left_arm_port,
                    "right_can_port": right_arm_port,
                    "left_effector_type": profile.effector_type_for("left") or "agx_gripper",
                    "right_effector_type": profile.effector_type_for("right") or "agx_gripper",
                    "firmware_version": profile.firmware_version,
                    "log_level": log_level,
                }.items(),
                condition=IfCondition(start_arm),
            )
        ]

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("agx_arm_ctrl"),
                    "launch",
                    "teleop_double_arm.launch.py",
                ]
            )
        ),
        launch_arguments={"profile_path": profile_path}.items(),
        condition=IfCondition(start_teleop),
    )

    def build_bridges(context):
        backend_val = backend.perform(context)
        bridges = []
        for side, port_name in (("left", left_port_name), ("right", right_port_name)):
            wrapper = bridge_wrapper_name("nero", side, backend_val)
            bridges.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [FindPackageShare("uma_bringup"), "launch", wrapper]
                        )
                    ),
                    launch_arguments={
                        "port_name": port_name,
                        "gripper_width_min": LaunchConfiguration("gripper_width_min"),
                        "gripper_width_max": LaunchConfiguration("gripper_width_max"),
                        "pub_rate": LaunchConfiguration("pub_rate"),
                        "log_level": log_level,
                        "profile_path": profile_path,
                    }.items(),
                )
            )
        return bridges

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_arm_port", default_value="left_arm"),
            DeclareLaunchArgument("right_arm_port", default_value="right_arm"),
            DeclareLaunchArgument("left_port_name", default_value="left_uma"),
            DeclareLaunchArgument("right_port_name", default_value="right_uma"),
            DeclareLaunchArgument(
                "backend", default_value="cpp", choices=["cpp", "python"],
                description="Handle bridge backend.",
            ),
            DeclareLaunchArgument("gripper_width_min", default_value="0.00"),
            DeclareLaunchArgument("gripper_width_max", default_value="0.07"),
            DeclareLaunchArgument("pub_rate", default_value="200.0"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "profile_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("uma_bringup"), "config", "nero_profile.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "start_arm", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "start_teleop", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "start_recording_control", default_value="true", choices=["true", "false"]
            ),
            recording_control_node(),
            OpaqueFunction(function=build_arm),
            teleop_launch,
            OpaqueFunction(function=build_bridges),
        ]
    )
