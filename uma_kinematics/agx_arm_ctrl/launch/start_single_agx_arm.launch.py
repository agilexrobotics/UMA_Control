"""Start one AGX arm controller node.

This is the canonical driver launch file.  Higher-level bringup files should
include it instead of constructing ``agx_arm_ctrl_single`` directly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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


def _topic_name(base, suffix):
    return [base, suffix]


def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level")
    topics_end = LaunchConfiguration("topics_end")

    driver = Node(
        package="agx_arm_ctrl",
        executable="agx_arm_ctrl_single",
        name=LaunchConfiguration("node_name"),
        prefix="python3",
        output="screen",
        ros_arguments=["--log-level", log_level],
        parameters=[
            {
                "can_port": LaunchConfiguration("can_port"),
                "arm_type": LaunchConfiguration("arm_type"),
                "effector_type": LaunchConfiguration("effector_type"),
                "auto_enable": LaunchConfiguration("auto_enable"),
                "speed_percent": LaunchConfiguration("speed_percent"),
                "pub_rate": LaunchConfiguration("pub_rate"),
                "enable_timeout": LaunchConfiguration("enable_timeout"),
                "installation_pos": LaunchConfiguration("installation_pos"),
                "firmware_version": LaunchConfiguration("firmware_version"),
                "payload": LaunchConfiguration("payload"),
                "tcp_offset": LaunchConfiguration("tcp_offset"),
                "publish_gripper_joint": LaunchConfiguration("publish_gripper_joint"),
            }
        ],
        remappings=[
            (
                "/feedback/joint_states",
                _topic_name("/feedback/joint_states", topics_end),
            ),
            ("/feedback/tcp_pose", _topic_name("/feedback/tcp_pose", topics_end)),
            ("/feedback/arm_status", _topic_name("/feedback/arm_status", topics_end)),
            (
                "/feedback/arm_ctrl_states",
                _topic_name("/feedback/arm_ctrl_states", topics_end),
            ),
            (
                "/feedback/gripper_status",
                _topic_name("/feedback/gripper_status", topics_end),
            ),
            ("/feedback/hand_status", _topic_name("/feedback/hand_status", topics_end)),
            ("/control/joint_states", _topic_name("/control/joint_states", topics_end)),
            ("/control/move_j", _topic_name("/control/move_j", topics_end)),
            ("/control/move_p", _topic_name("/control/move_p", topics_end)),
            ("/control/move_l", _topic_name("/control/move_l", topics_end)),
            ("/control/move_c", _topic_name("/control/move_c", topics_end)),
            ("/control/move_mit", _topic_name("/control/move_mit", topics_end)),
            ("/control/move_js", _topic_name("/control/move_js", topics_end)),
            ("/control/gripper", _topic_name("/control/gripper", topics_end)),
            ("/control/hand", _topic_name("/control/hand", topics_end)),
            (
                "/control/hand_position_time",
                _topic_name("/control/hand_position_time", topics_end),
            ),
            ("/enable_agx_arm", _topic_name("/enable_agx_arm", topics_end)),
            ("/move_home", _topic_name("/move_home", topics_end)),
            ("/exit_teach_mode", _topic_name("/exit_teach_mode", topics_end)),
        ],
    )

    return LaunchDescription(
        [
            _arg("can_port", "can0", "SocketCAN interface connected to the arm."),
            _arg(
                "arm_type",
                "piper",
                "Arm model supported by the pyAgxArm driver.",
                ARM_TYPES,
            ),
            _arg(
                "effector_type",
                "none",
                "End-effector implementation.",
                EFFECTOR_TYPES,
            ),
            _arg(
                "auto_enable",
                "true",
                "Automatically enable the arm after connecting.",
                BOOL_CHOICES,
            ),
            _arg(
                "installation_pos",
                "horizontal",
                "Arm installation orientation used for gravity compensation.",
                INSTALLATION_POSITIONS,
            ),
            _arg(
                "firmware_version",
                "default",
                "pyAgxArm driver firmware profile; must match the arm firmware.",
                FIRMWARE_VERSIONS,
            ),
            _arg(
                "speed_percent",
                "100",
                "Motion speed as a percentage of the driver maximum.",
            ),
            _arg("pub_rate", "200.0", "Feedback publication rate in Hz."),
            _arg(
                "enable_timeout",
                "5.0",
                "Timeout in seconds for enable/disable operations.",
            ),
            _arg("payload", "empty", "Payload preset.", PAYLOAD_TYPES),
            _arg(
                "tcp_offset",
                "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                "TCP offset [x, y, z, roll, pitch, yaw] in m/rad.",
            ),
            _arg(
                "publish_gripper_joint",
                "true",
                "Publish the aggregate gripper joint in feedback/joint_states.",
                BOOL_CHOICES,
            ),
            _arg(
                "node_name",
                "agx_arm_ctrl_single_node",
                "Unique ROS node name.",
            ),
            _arg(
                "topics_end",
                "",
                "Suffix appended to every driver topic and service; use _left/_right for dual-arm operation.",
            ),
            _arg("log_level", "info", "ROS logging level."),
            driver,
        ]
    )
