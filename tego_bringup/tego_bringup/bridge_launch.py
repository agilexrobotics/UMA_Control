"""Shared launch construction for single-arm handle bridges."""

from pathlib import Path
from typing import Dict

from agx_arm_ctrl.robot_profile import format_arm_home_state, load_profile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_HANDLE_SEGMENTS = {
    "handle_seg0": [0.0, 0.0, 0.0534, 0.0, 0.0, 0.0],
    "handle_seg1": [0.0, 0.0, 0.0, 1.5707963, 0.0, 3.1415926],
    "handle_seg2": [0.220, 0.0, 0.0, 0.0, 0.0, -1.5707963],
    "handle_seg3": [-0.04804, -0.19115, 0.05487, 1.5707963, 0.0, 0.0],
}

# nero 手柄 wrapper 用实例名 "arm"(left_arm.launch.py),piper 用机型名
_BRIDGE_WRAPPER_MODEL = {"nero": "arm", "piper": "piper"}


def bridge_wrapper_name(arm_type: str, side: str, backend: str) -> str:
    """Wrapper launch file name for a single handle bridge.

    e.g. ("nero", "left", "cpp") -> "left_arm.launch.py"
         ("piper", "right", "python") -> "right_piper_python.launch.py"
    """
    model = _BRIDGE_WRAPPER_MODEL[arm_type]
    suffix = "" if backend == "cpp" else "_python"
    return f"{side}_{model}{suffix}.launch.py"


def _side_values(side: str, arm_type: str) -> Dict[str, object]:
    if side not in ("left", "right"):
        raise ValueError(f"unsupported arm side: {side}")
    if arm_type not in ("nero", "piper"):
        raise ValueError(f"unsupported bridge arm type: {arm_type}")

    if arm_type == "nero":
        world_to_base_y = 0.15 if side == "left" else -0.15
        gripper_length = 0.0
    else:
        world_to_base_y = 0.1 if side == "left" else -0.15
        gripper_length = 0.14

    values = {
        "node_name": f"tego_{side[0]}",
        "angle_reverse": side == "right",
        "pika_pose_topic": f"/{side}_handle_pose",
        "remote_joint_topic_in": f"/control/joint_states_{side}_raw",
        "remote_joint_topic_out": f"/control/move_js_{side}",
        "remote_joint_single_topic": f"/feedback/joint_states_{side}",
        "teleop_enable_topic": f"/{side}_teleop_enable",
        "remote_service_name": f"{side}_teleop_trigger",
        "world_to_base_y": world_to_base_y,
        "is_nero": arm_type == "nero",
        "nero_gripper_topic_name": f"/control/joint_states_{side}",
        "use_piper_mode": arm_type == "piper",
        "gripper_length": gripper_length,
        "default_profile": f"{arm_type}_profile.yaml",
        "default_zoom_factor": 1.4 if arm_type == "nero" else 1.1,
        "default_effector_type": "agx_gripper" if arm_type == "nero" else "none",
    }
    values.update(_HANDLE_SEGMENTS)
    return values


def _profile_values(context, profile_path, side, arm_type):
    values = _side_values(side, arm_type)
    path = Path(profile_path.perform(context))
    profile = load_profile(str(path), validate_robot=False)
    values["arm_home_state"] = format_arm_home_state(
        profile, side
    )
    values["zoom_factor"] = float(
        (profile.handle or {}).get("zoom_factor", values.pop("default_zoom_factor"))
    )
    values["effector_type"] = (
        profile.effector_type_for(side) or values.pop("default_effector_type")
    )
    values.pop("default_profile")
    return values


def _bridge_parameters(context, side, arm_type, substitutions):
    values = _profile_values(
        context,
        substitutions["profile_path"],
        side,
        arm_type,
    )
    values.update(
        {
            "port_name": substitutions["port_name"],
            "pub_rate": substitutions["pub_rate"],
            "log_level": substitutions["log_level"],
            "gripper_width_min": substitutions["gripper_width_min"],
            "gripper_width_max": substitutions["gripper_width_max"],
        }
    )
    return values


def _cpp_bridge(context, side, arm_type, substitutions):
    values = _bridge_parameters(context, side, arm_type, substitutions)
    run_single_node = PathJoinSubstitution(
        [FindPackageShare("tego_bringup"), "launch", "run_single_node.launch.py"]
    )
    launch_values = {
        "port_name": values["port_name"],
        "node_name": values["node_name"],
        "angle_reverse": str(values["angle_reverse"]).lower(),
        "pika_pose_topic": values["pika_pose_topic"],
        "remote_joint_topic_in": values["remote_joint_topic_in"],
        "remote_joint_topic_out": values["remote_joint_topic_out"],
        "remote_joint_single_topic": values["remote_joint_single_topic"],
        "teleop_enable_topic": values["teleop_enable_topic"],
        "remote_service_name": values["remote_service_name"],
        "world_to_base_y": str(values["world_to_base_y"]),
        "is_nero": str(values["is_nero"]).lower(),
        "nero_gripper_topic_name": values["nero_gripper_topic_name"],
        "effector_type": values["effector_type"],
        "arm_home_state": values["arm_home_state"],
        "publish_sim_joint": "false",
        "gripper_width_min": values["gripper_width_min"],
        "gripper_width_max": values["gripper_width_max"],
        "pub_rate": values["pub_rate"],
        "log_level": values["log_level"],
        "use_piper_mode": str(values["use_piper_mode"]).lower(),
        "handle_seg0": str(values["handle_seg0"]),
        "handle_seg1": str(values["handle_seg1"]),
        "handle_seg2": str(values["handle_seg2"]),
        "handle_seg3": str(values["handle_seg3"]),
        "use_pose_dt_zoom": "true",
        "zoom_factor": str(values["zoom_factor"]),
        "use_trigger_pose": "true",
        "gripper_length": str(values["gripper_length"]),
    }
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(run_single_node),
        launch_arguments=launch_values.items(),
    )


def _python_bridge(context, side, arm_type, substitutions):
    values = _bridge_parameters(context, side, arm_type, substitutions)
    parameters = {
        "port_name": values["port_name"],
        "pub_rate": values["pub_rate"],
        "node_name": values["node_name"],
        "base_to_imu_roll": 93.0,
        "base_to_imu_pitch": 5.0,
        "base_to_imu_yaw": -6.0,
        "imu_reverse": False,
        "add_angle": False,
        "double_press_timeout": 0.30,
        "show_imu_data": False,
        "angle_reverse": values["angle_reverse"],
        "pika_pose_topic": values["pika_pose_topic"],
        "remote_joint_topic_in": values["remote_joint_topic_in"],
        "remote_joint_topic_out": values["remote_joint_topic_out"],
        "remote_joint_single_topic": values["remote_joint_single_topic"],
        "teleop_enable_topic": values["teleop_enable_topic"],
        "remote_service_name": values["remote_service_name"],
        "world_to_base_y": values["world_to_base_y"],
        "is_nero": values["is_nero"],
        "nero_gripper_topic_name": values["nero_gripper_topic_name"],
        "effector_type": values["effector_type"],
        "arm_home_state": values["arm_home_state"],
        "publish_sim_joint": False,
        "gripper_width_min": values["gripper_width_min"],
        "gripper_width_max": values["gripper_width_max"],
        "log_level": values["log_level"],
        "use_piper_mode": values["use_piper_mode"],
        "handle_seg0": values["handle_seg0"],
        "handle_seg1": values["handle_seg1"],
        "handle_seg2": values["handle_seg2"],
        "handle_seg3": values["handle_seg3"],
        "use_pose_dt_zoom": True,
        "zoom_factor": values["zoom_factor"],
        "use_trigger_pose": True,
        "gripper_length": values["gripper_length"],
    }
    return Node(
        package="tego_ros",
        executable="tego_ros_py_node.py",
        name=values["node_name"],
        parameters=[parameters],
        ros_arguments=["--log-level", values["log_level"]],
        output="screen",
        remappings=[
            ("/pika_pose", values["pika_pose_topic"]),
            ("/joint_states_single", values["remote_joint_single_topic"]),
            ("/teleop_enable", values["teleop_enable_topic"]),
        ],
    )


def make_single_bridge_launch(arm_type: str, side: str, backend: str) -> LaunchDescription:
    if arm_type not in ("nero", "piper"):
        raise ValueError(f"unsupported arm type: {arm_type}")
    if side not in ("left", "right"):
        raise ValueError(f"unsupported arm side: {side}")
    if backend not in ("cpp", "python"):
        raise ValueError(f"unsupported bridge backend: {backend}")

    profile_default = PathJoinSubstitution(
        [FindPackageShare("tego_bringup"), "config", f"{arm_type}_profile.yaml"]
    )
    substitutions = {
        "profile_path": LaunchConfiguration("profile_path"),
        "port_name": LaunchConfiguration("port_name"),
        "gripper_width_min": LaunchConfiguration("gripper_width_min"),
        "gripper_width_max": LaunchConfiguration("gripper_width_max"),
        "pub_rate": LaunchConfiguration("pub_rate"),
        "log_level": LaunchConfiguration("log_level"),
    }

    def build(context):
        factory = _cpp_bridge if backend == "cpp" else _python_bridge
        return [factory(context, side, arm_type, substitutions)]

    return LaunchDescription(
        [
            DeclareLaunchArgument("profile_path", default_value=profile_default),
            DeclareLaunchArgument("port_name", default_value=f"{side}_tego"),
            DeclareLaunchArgument("gripper_width_min", default_value="0.00"),
            DeclareLaunchArgument("gripper_width_max", default_value="0.07"),
            DeclareLaunchArgument("pub_rate", default_value="200.0"),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=build),
        ]
    )
