from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        default_value="piper",
        choices=["nero", "piper", "piper_h", "piper_l", "piper_x"],
        description="Robotic arm type (e.g. nero, piper, piper_h, piper_l, piper_x).",
    )

    effector_type_arg = DeclareLaunchArgument(
        "effector_type",
        default_value="none",
        choices=["none", "agx_gripper", "revo2"],
        description="End effector type (e.g. agx_gripper, revo2).",
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

    tcp_offset_arg = DeclareLaunchArgument(
        "tcp_offset",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description="TCP offset in x, y, z, roll, pitch, yaw in meters/radians.",
    )

    publish_gripper_joint_arg = DeclareLaunchArgument(
        "publish_gripper_joint",
        default_value="true",
        choices=["true", "false"],
        description='Publish "gripper" (opening width) joint in /feedback/joint_states. '
        "Set false when used with MoveIt (URDF only has gripper_joint1/2).",
    )

    node_name_arg = DeclareLaunchArgument(
        "node_name", default_value="agx_arm_ctrl_single_node", description=""
    )

    topics_end_arg = DeclareLaunchArgument(
        "topics_end", default_value="", description=""
    )

    # node
    agx_arm_node = Node(
        package="agx_arm_ctrl",
        executable="agx_arm_ctrl_single",
        name=LaunchConfiguration("node_name"),
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {
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
                "publish_gripper_joint": LaunchConfiguration("publish_gripper_joint"),
            }
        ],
        remappings=[
            (
                "/feedback/joint_states",
                ["/feedback/joint_states", LaunchConfiguration("topics_end")],
            ),
            (
                "/feedback/tcp_pose",
                ["/feedback/tcp_pose", LaunchConfiguration("topics_end")],
            ),
            (
                "/feedback/arm_status",
                ["/feedback/arm_status", LaunchConfiguration("topics_end")],
            ),
            (
                "/feedback/arm_ctrl_states",
                ["/feedback/arm_ctrl_states", LaunchConfiguration("topics_end")],
            ),
            (
                "/feedback/gripper_status",
                ["/feedback/gripper_status", LaunchConfiguration("topics_end")],
            ),
            (
                "/feedback/hand_status",
                ["/feedback/hand_status", LaunchConfiguration("topics_end")],
            ),
            # control topics
            (
                "/control/joint_states",
                ["/control/joint_states", LaunchConfiguration("topics_end")],
            ),  # move_js joint_states
            ("/control/move_j", ["/control/move_j", LaunchConfiguration("topics_end")]),
            ("/control/move_p", ["/control/move_p", LaunchConfiguration("topics_end")]),
            ("/control/move_l", ["/control/move_l", LaunchConfiguration("topics_end")]),
            ("/control/move_c", ["/control/move_c", LaunchConfiguration("topics_end")]),
            (
                "/control/move_mit",
                ["/control/move_mit", LaunchConfiguration("topics_end")],
            ),
            (
                "/control/move_js",
                ["/control/move_js", LaunchConfiguration("topics_end")],
            ),
            (
                "/control/gripper",
                ["/control/gripper", LaunchConfiguration("topics_end")],
            ),
            ("/control/hand", ["/control/hand", LaunchConfiguration("topics_end")]),
            (
                "/control/hand_position_time",
                ["/control/hand_position_time", LaunchConfiguration("topics_end")],
            ),
            # services
            ("/enable_agx_arm", ["/enable_agx_arm", LaunchConfiguration("topics_end")]),
            ("/move_home", ["/move_home", LaunchConfiguration("topics_end")]),
            (
                "/exit_teach_mode",
                ["/exit_teach_mode", LaunchConfiguration("topics_end")],
            ),
        ],
    )

    return LaunchDescription(
        [
            # arguments
            log_level_arg,
            can_port_arg,
            arm_type_arg,
            effector_type_arg,
            auto_enable_arg,
            installation_pos_arg,
            speed_percent_arg,
            pub_rate_arg,
            enable_timeout_arg,
            payload_arg,
            tcp_offset_arg,
            publish_gripper_joint_arg,
            node_name_arg,
            topics_end_arg,
            # node
            agx_arm_node,
        ]
    )
