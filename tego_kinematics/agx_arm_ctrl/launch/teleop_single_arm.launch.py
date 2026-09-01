"""Start the profile-driven teleoperation node for one arm side."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "side",
                default_value="left",
                choices=["left", "right"],
                description="Arm side whose feedback and handle topics are used.",
            ),
            DeclareLaunchArgument(
                "profile_path",
                description="YAML robot profile used by the IK teleoperator.",
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="teleop_single_arm_node",
                description="Unique ROS node name.",
            ),
            DeclareLaunchArgument(
                "feedback_timeout_sec",
                default_value="1.0",
                description="Feedback timeout before teleoperation stops publishing.",
            ),
            Node(
                package="agx_arm_ctrl",
                executable="teleop_single_arm",
                name=LaunchConfiguration("node_name"),
                prefix="python3",
                parameters=[
                    {
                        "side": LaunchConfiguration("side"),
                        "profile_path": LaunchConfiguration("profile_path"),
                        "feedback_timeout_sec": LaunchConfiguration(
                            "feedback_timeout_sec"
                        ),
                    }
                ],
                additional_env={"PYTHONNOUSERSITE": "1"},
                output="screen",
            ),
        ]
    )
