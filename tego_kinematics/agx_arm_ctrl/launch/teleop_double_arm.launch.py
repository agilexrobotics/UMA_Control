"""Start profile-driven teleoperation nodes for both arm sides."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _teleop_include(side, node_name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("agx_arm_ctrl"), "launch", "teleop_single_arm.launch.py"]
            )
        ),
        launch_arguments={
            "side": side,
            "node_name": node_name,
            "profile_path": LaunchConfiguration("profile_path"),
            "feedback_timeout_sec": LaunchConfiguration("feedback_timeout_sec"),
        }.items(),
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile_path", description="Robot profile YAML path."),
            DeclareLaunchArgument(
                "feedback_timeout_sec",
                default_value="1.0",
                description="Feedback timeout in seconds for both teleoperators.",
            ),
            _teleop_include("left", "teleop_left_arm_node"),
            _teleop_include("right", "teleop_right_arm_node"),
        ]
    )
