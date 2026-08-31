from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def generate_launch_description():

    # arg
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal).",
    )

    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value="can0",
        description="CAN port to be used by the AGX Arm node.",
    )

    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="nero",
        choices=["nero", "piper", "piper_h", "piper_l", "piper_x"],
        description="Robotic arm type (e.g. nero, piper, piper_h, piper_l, piper_x).",
    )

    effector_type_arg = DeclareLaunchArgument(
        "effector_type",
        default_value="none",
        choices=["none", "agx_gripper", "revo2"],
        description="End effector type (e.g. agx_gripper, revo2).",
    )

    revo2_type_arg = DeclareLaunchArgument(
        "revo2_type",
        default_value="left",
        choices=["left", "right"],
        description="Revo2 end effector type (e.g. left, right).",
    )

    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable",
        default_value="true",
        choices=["true", "false"],
        description="Automatically enable the AGX Arm node.",
    )

    installation_pos_arg = DeclareLaunchArgument(
        "installation_pos",
        default_value="horizontal",
        choices=["horizontal", "left", "right"],
        description="Installation position of the arm (e.g. horizontal, left, right).",
    )

    speed_percent_arg = DeclareLaunchArgument(
        "speed_percent",
        default_value="30",
        description="Movement speed as a percentage of maximum speed.",
    )

    pub_rate_arg = DeclareLaunchArgument(
        "pub_rate",
        default_value="200",
        description="Publishing rate for the AGX Arm node.",
    )

    enable_timeout_arg = DeclareLaunchArgument(
        "enable_timeout",
        default_value="5.0",
        description="Timeout in seconds for arm enable/disable operations.",
    )

    payload_arg = DeclareLaunchArgument(
        "payload",
        default_value="empty",
        choices=["empty", "half", "full"],
        description="Payload type (e.g. empty, half, full).",
    )

    follow_arg = DeclareLaunchArgument(
        "follow",
        default_value="true",
        choices=["true", "false"],
        description="Follow real arm state.",
    )

    control_arg = DeclareLaunchArgument(
        "control",
        default_value="false",
        choices=["true", "false"],
        description="Whether to publish control topics from the RViz-side joint state publisher.",
    )

    tcp_offset_arg = DeclareLaunchArgument(
        "tcp_offset",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description="TCP offset in x, y, z, roll, pitch, yaw in meters/radians.",
    )

    # description:统一使用 tego_description/launch/display.launch.py
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tego_description"),
                "launch",
                "display.launch.py",
            )
        ),
        launch_arguments={
            "arm_type": LaunchConfiguration("arm_type"),
            "effector_type": LaunchConfiguration("effector_type"),
            "revo2_type": LaunchConfiguration("revo2_type"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "follow": LaunchConfiguration("follow"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "control": LaunchConfiguration("control"),
        }.items(),
    )

    # agx_arm
    agx_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_ctrl"),
                "launch",
                "start_single_agx_arm.launch.py",
            )
        ),
        launch_arguments={
            "can_port": LaunchConfiguration("can_port"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "arm_type": LaunchConfiguration("arm_type"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "installation_pos": LaunchConfiguration("installation_pos"),
            "effector_type": LaunchConfiguration("effector_type"),
            "payload": LaunchConfiguration("payload"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
        }.items(),
    )

    return LaunchDescription(
        [
            # arguments
            log_level_arg,
            can_port_arg,
            arm_type_arg,
            effector_type_arg,
            revo2_type_arg,
            auto_enable_arg,
            installation_pos_arg,
            speed_percent_arg,
            pub_rate_arg,
            enable_timeout_arg,
            payload_arg,
            tcp_offset_arg,
            follow_arg,
            control_arg,
            # description
            description_launch,
            # agx_arm
            agx_arm_launch,
        ]
    )
