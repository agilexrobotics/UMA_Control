"""Start one AGX arm controller together with the shared RViz description."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


ARM_TYPES = ["nero", "piper", "piper_h", "piper_l", "piper_x"]
EFFECTOR_TYPES = ["none", "agx_gripper", "revo2"]
INSTALLATION_POSITIONS = ["horizontal", "left", "right"]
FIRMWARE_VERSIONS = ["default", "v111", "v112", "v120", "v121", "v183", "v188", "v189"]
PAYLOAD_TYPES = ["empty", "half", "full"]
BOOL_CHOICES = ["true", "false"]


def _arg(name, default, description, choices=None):
    kwargs = {"default_value": default, "description": description}
    if choices is not None:
        kwargs["choices"] = choices
    return DeclareLaunchArgument(name, **kwargs)


def generate_launch_description() -> LaunchDescription:
    arm_type = LaunchConfiguration("arm_type")
    effector_type = LaunchConfiguration("effector_type")
    tcp_offset = LaunchConfiguration("tcp_offset")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("tego_description"), "launch", "display.launch.py"]
            )
        ),
        launch_arguments={
            "arm_type": arm_type,
            "custom_model": LaunchConfiguration("custom_model"),
            "effector_type": effector_type,
            "revo2_type": LaunchConfiguration("revo2_type"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "gui": LaunchConfiguration("gui"),
            "rvizconfig": LaunchConfiguration("rvizconfig"),
            "follow": LaunchConfiguration("follow"),
            "control": LaunchConfiguration("control"),
            "tcp_offset": tcp_offset,
        }.items(),
    )

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("agx_arm_ctrl"),
                    "launch",
                    "start_single_agx_arm.launch.py",
                ]
            )
        ),
        launch_arguments={
            "can_port": LaunchConfiguration("can_port"),
            "arm_type": arm_type,
            "effector_type": effector_type,
            "auto_enable": LaunchConfiguration("auto_enable"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "pub_rate": LaunchConfiguration("pub_rate"),
            "enable_timeout": LaunchConfiguration("enable_timeout"),
            "installation_pos": LaunchConfiguration("installation_pos"),
            "firmware_version": LaunchConfiguration("firmware_version"),
            "payload": LaunchConfiguration("payload"),
            "tcp_offset": tcp_offset,
            "publish_gripper_joint": LaunchConfiguration("publish_gripper_joint"),
            "node_name": LaunchConfiguration("node_name"),
            "topics_end": LaunchConfiguration("topics_end"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    return LaunchDescription(
        [
            _arg("can_port", "can0", "SocketCAN interface connected to the arm."),
            _arg("arm_type", "nero", "Arm model.", ARM_TYPES),
            _arg("effector_type", "none", "End-effector implementation.", EFFECTOR_TYPES),
            _arg(
                "auto_enable",
                "true",
                "Automatically enable the arm.",
                BOOL_CHOICES,
            ),
            _arg(
                "installation_pos",
                "horizontal",
                "Arm installation orientation.",
                INSTALLATION_POSITIONS,
            ),
            _arg(
                "firmware_version",
                "default",
                "pyAgxArm driver firmware profile; must match the arm firmware.",
                FIRMWARE_VERSIONS,
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
                "Publish aggregate gripper joint feedback.",
                BOOL_CHOICES,
            ),
            _arg("node_name", "agx_arm_ctrl_single_node", "Unique driver node name."),
            _arg("topics_end", "", "Suffix for driver topics and services."),
            _arg("log_level", "info", "ROS logging level."),
            _arg("custom_model", "", "Optional custom URDF/Xacro path."),
            _arg("revo2_type", "left", "Revo2 hand side.", ["left", "right"]),
            _arg("gui", "true", "Enable joint_state_publisher_gui.", BOOL_CHOICES),
            _arg("rvizconfig", "", "RViz config path; empty uses package default."),
            _arg("follow", "true", "Follow real arm feedback.", BOOL_CHOICES),
            _arg("control", "false", "Enable RViz joint-state control.", BOOL_CHOICES),
            description,
            driver,
        ]
    )
