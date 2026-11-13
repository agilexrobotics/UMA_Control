from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from uma_bringup.bridge_launch import recording_control_node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("left_arm_port", default_value="left_arm"),
        DeclareLaunchArgument("right_arm_port", default_value="right_arm"),
        DeclareLaunchArgument("left_port_name", default_value="left_uma"),
        DeclareLaunchArgument("right_port_name", default_value="right_uma"),
        DeclareLaunchArgument("backend", default_value="cpp", choices=["cpp", "python"]),
        DeclareLaunchArgument(
            "profile_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("uma_bringup"), "config", "piper_x_profile.yaml"]
            ),
        ),
        DeclareLaunchArgument("gripper_width_min", default_value="0.00"),
        DeclareLaunchArgument("gripper_width_max", default_value="0.07"),
        DeclareLaunchArgument("pub_rate", default_value="200.0"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("start_arm", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_teleop", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "start_recording_control", default_value="true", choices=["true", "false"]
        ),
    ]
    arms = []
    for side in ("left", "right"):
        values = {
            name: LaunchConfiguration(name)
            for name in (
                "backend", "profile_path", "gripper_width_min", "gripper_width_max",
                "pub_rate", "log_level", "start_arm", "start_teleop",
            )
        }
        values.update(
            arm_type="piper_x",
            side=side,
            arm_port=LaunchConfiguration(f"{side}_arm_port"),
            port_name=LaunchConfiguration(f"{side}_port_name"),
            start_recording_control="false",
        )
        # 两侧参数分别生效，采集控制由双臂入口统一启动。
        arms.append(
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [FindPackageShare("uma_bringup"), "launch", "demo_piper.launch.py"]
                            )
                        ),
                        launch_arguments=values.items(),
                    )
                ],
            )
        )
    return LaunchDescription([*arguments, recording_control_node(), *arms])
