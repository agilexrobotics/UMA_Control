"""Piper single-arm demo: arm driver + Pinocchio teleop + handle bridge.

一个入口覆盖全部组合:``side:=left/right``(默认 right)、``backend:=cpp/python``(默认 cpp)。
安装朝向从 profile 读取(``robot.installation_pos``)。
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from agx_arm_ctrl.robot_profile import load_profile
from tego_bringup.bridge_launch import bridge_wrapper_name


def generate_launch_description() -> LaunchDescription:
    arm_port = LaunchConfiguration("arm_port")
    port_name = LaunchConfiguration("port_name")
    gripper_width_min = LaunchConfiguration("gripper_width_min")
    gripper_width_max = LaunchConfiguration("gripper_width_max")
    pub_rate = LaunchConfiguration("pub_rate")
    log_level = LaunchConfiguration("log_level")
    profile_path = LaunchConfiguration("profile_path")
    side = LaunchConfiguration("side")
    backend = LaunchConfiguration("backend")
    start_arm = LaunchConfiguration("start_arm")
    start_teleop = LaunchConfiguration("start_teleop")

    def build(context):
        side_val = side.perform(context)
        backend_val = backend.perform(context)
        # 安装朝向来自 profile(piper 专属):horizontal/left/right
        installation_pos = "horizontal"
        try:
            profile = load_profile(str(Path(profile_path.perform(context))), validate_robot=False)
            installation_pos = profile.installation_pos
        except Exception as e:  # launch 期解析失败不阻塞整栈,沿用默认
            print(f"[demo_piper] 读取 installation_pos 失败,默认 horizontal: {e}")
        wrapper = bridge_wrapper_name("piper", side_val, backend_val)
        return [
            IncludeLaunchDescription(
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
                    "can_port": arm_port,
                    "arm_type": "piper",
                    "installation_pos": installation_pos,
                    "topics_end": f"_{side_val}",
                    "log_level": log_level,
                }.items(),
                condition=IfCondition(start_arm),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("agx_arm_ctrl"),
                            "launch",
                            f"teleop_{side_val}_arm.launch.py",
                        ]
                    )
                ),
                launch_arguments={"profile_path": profile_path}.items(),
                condition=IfCondition(start_teleop),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("tego_bringup"), "launch", wrapper]
                    )
                ),
                launch_arguments={
                    "port_name": port_name,
                    "profile_path": profile_path,
                    "gripper_width_min": gripper_width_min,
                    "gripper_width_max": gripper_width_max,
                    "pub_rate": pub_rate,
                    "log_level": log_level,
                }.items(),
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_port", default_value="right_arm",
                description="SocketCAN interface of the Piper arm.",
            ),
            DeclareLaunchArgument(
                "port_name", default_value="right_tego",
                description="SocketCAN interface of the handle.",
            ),
            DeclareLaunchArgument(
                "side", default_value="right", choices=["left", "right"],
                description="Arm/handle side.",
            ),
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
                    [FindPackageShare("tego_bringup"), "config", "piper_profile.yaml"]
                ),
            ),
            DeclareLaunchArgument("start_arm", default_value="true"),
            DeclareLaunchArgument("start_teleop", default_value="true"),
            OpaqueFunction(function=build),
        ]
    )
