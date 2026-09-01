"""Start two independent AGX arm controller nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


ARM_TYPES = ["nero", "piper", "piper_h", "piper_l", "piper_x"]
EFFECTOR_TYPES = ["none", "agx_gripper", "revo2"]
INSTALLATION_POSITIONS = ["horizontal", "left", "right"]
PAYLOAD_TYPES = ["empty", "half", "full"]
BOOL_CHOICES = ["true", "false"]


def _arg(name, default, description, choices=None):
    kwargs = {"default_value": default, "description": description}
    if choices is not None:
        kwargs["choices"] = choices
    return DeclareLaunchArgument(name, **kwargs)


def _arm_include(side, can_port, node_name, installation_pos, topics_end):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("agx_arm_ctrl"), "launch", "start_single_agx_arm.launch.py"]
            )
        ),
        launch_arguments={
            "can_port": can_port,
            "arm_type": LaunchConfiguration("arm_type"),
            "effector_type": LaunchConfiguration("effector_type"),
            "auto_enable": LaunchConfiguration("auto_enable"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "installation_pos": installation_pos,
            "payload": LaunchConfiguration("payload"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "publish_gripper_joint": LaunchConfiguration("publish_gripper_joint"),
            "node_name": node_name,
            "topics_end": topics_end,
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            _arg("left_can_port", "left_arm", "SocketCAN interface for the left arm."),
            _arg("right_can_port", "right_arm", "SocketCAN interface for the right arm."),
            _arg("arm_type", "nero", "Arm model used by both controllers.", ARM_TYPES),
            _arg(
                "effector_type",
                "agx_gripper",
                "End-effector implementation used by both controllers.",
                EFFECTOR_TYPES,
            ),
            _arg(
                "auto_enable",
                "true",
                "Automatically enable both arms.",
                BOOL_CHOICES,
            ),
            _arg(
                "left_installation_pos",
                "horizontal",
                "Installation orientation for the left arm.",
                INSTALLATION_POSITIONS,
            ),
            _arg(
                "right_installation_pos",
                "horizontal",
                "Installation orientation for the right arm.",
                INSTALLATION_POSITIONS,
            ),
            _arg("speed_percent", "100", "Motion speed percentage."),
            _arg("pub_rate", "200.0", "Feedback publication rate in Hz."),
            _arg("enable_timeout", "5.0", "Enable/disable timeout in seconds."),
            _arg("payload", "empty", "Payload preset.", PAYLOAD_TYPES),
            _arg(
                "tcp_offset",
                "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                "TCP offset [x, y, z, roll, pitch, yaw] in m/rad.",
            ),
            _arg(
                "publish_gripper_joint",
                "true",
                "Publish aggregate gripper joints.",
                BOOL_CHOICES,
            ),
            _arg("left_node_name", "agx_arm_ctrl_left", "Left controller node name."),
            _arg("right_node_name", "agx_arm_ctrl_right", "Right controller node name."),
            _arg("log_level", "info", "ROS logging level."),
            _arm_include(
                "left",
                LaunchConfiguration("left_can_port"),
                LaunchConfiguration("left_node_name"),
                LaunchConfiguration("left_installation_pos"),
                "_left",
            ),
            _arm_include(
                "right",
                LaunchConfiguration("right_can_port"),
                LaunchConfiguration("right_node_name"),
                LaunchConfiguration("right_installation_pos"),
                "_right",
            ),
        ]
    )
