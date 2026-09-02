"""Launch the RViz interactive safety-space editor with the profile URDF."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _editable_profile_dirs() -> list[Path]:
    """Return profile directories in editable and installed environments."""
    directories: list[Path] = []
    configured_dir = os.environ.get("TEGO_PROFILE_DIR")
    if configured_dir:
        directories.append(Path(configured_dir).expanduser())

    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        source_dir = parent / "tego_bringup" / "config"
        if source_dir.is_dir():
            directories.append(source_dir)
            break

    try:
        directories.append(get_package_share_path("tego_bringup") / "config")
    except LookupError:
        pass

    unique_directories: list[Path] = []
    for directory in directories:
        directory = directory.resolve()
        if directory not in unique_directories:
            unique_directories.append(directory)
    return unique_directories


def _resolve_profile_path(profile_name: str) -> Path:
    """Resolve a profile name, preferring the editable workspace copy."""
    profile = Path(profile_name).expanduser()
    if profile.is_absolute():
        return profile.resolve()

    directories = _editable_profile_dirs()
    for directory in directories:
        candidate = directory / profile
        if candidate.is_file():
            return candidate.resolve()

    searched = ", ".join(str(directory / profile) for directory in directories)
    raise ValueError(f"找不到 profile: {profile_name}，已搜索: {searched}")


def _profile_path_from_context(context) -> Path:
    """Resolve the selected profile after launch substitutions are evaluated."""
    profile_name = LaunchConfiguration("profile_name").perform(context)
    return _resolve_profile_path(profile_name)


def _resolve_resource_path(resource_uri: str, profile_path: Path) -> Path:
    """Resolve a profile resource path, including ROS ``package://`` URIs."""
    if resource_uri.startswith("package://"):
        package_and_path = resource_uri[len("package://"):]
        package_name, separator, relative_path = package_and_path.partition("/")
        if not package_name or not separator or not relative_path:
            raise ValueError(
                "robot.urdf_path 的 package:// URI 格式错误，"
                "应为 package://<package>/<relative-path>"
            )
        try:
            return get_package_share_path(package_name) / relative_path
        except LookupError as exc:
            raise ValueError(
                f"robot.urdf_path 引用的 ROS 包不存在: {package_name}"
            ) from exc

    if resource_uri.startswith("file://"):
        resource_uri = resource_uri[len("file://"):]

    resource_path = Path(resource_uri).expanduser()
    if not resource_path.is_absolute():
        resource_path = profile_path.parent / resource_path
    return resource_path


def _load_robot_description(profile_path: Path) -> str:
    """Read and return the URDF XML referenced by a robot profile."""
    try:
        with profile_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
    except OSError as exc:
        raise ValueError(f"无法读取 profile: {profile_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"profile YAML 格式错误: {profile_path}: {exc}") from exc

    robot = document.get("robot")
    if not isinstance(robot, dict):
        raise ValueError(f"profile 缺少 robot 配置: {profile_path}")

    urdf_uri = robot.get("urdf_path")
    if not isinstance(urdf_uri, str) or not urdf_uri.strip():
        raise ValueError(f"profile 缺少 robot.urdf_path: {profile_path}")

    urdf_path = _resolve_resource_path(urdf_uri.strip(), profile_path)
    if not urdf_path.is_file():
        raise ValueError(
            f"profile 中的 URDF 不存在: {urdf_uri} -> {urdf_path}"
        )

    if urdf_path.suffix.lower() == ".xacro":
        try:
            import xacro

            return xacro.process_file(str(urdf_path)).toxml()
        except ImportError as exc:
            raise ValueError(
                f"URDF 配置为 xacro，但当前环境没有 xacro: {urdf_path}"
            ) from exc
        except Exception as exc:
            raise ValueError(f"xacro 处理失败: {urdf_path}: {exc}") from exc

    try:
        robot_description = urdf_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"无法读取 URDF: {urdf_path}: {exc}") from exc
    if not robot_description.strip():
        raise ValueError(f"URDF 文件为空: {urdf_path}")
    return robot_description


def _robot_state_publisher_from_profile(context, *args, **kwargs):
    """Create robot_state_publisher after launch substitutions are resolved."""
    profile_path = _profile_path_from_context(context)
    robot_description = _load_robot_description(profile_path)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def _arm_safety_monitor_from_profile(context, *args, **kwargs):
    """Create the editor node with the same editable profile path."""
    profile_path = _profile_path_from_context(context)
    return [
        Node(
            package="arm_safety_monitor",
            executable="arm_safety_monitor_node",
            name="arm_safety_monitor",
            output="screen",
            parameters=[
                {
                    "profile_path": str(profile_path),
                    "box_resize_step": LaunchConfiguration("box_resize_step"),
                    "plane_size": LaunchConfiguration("plane_size"),
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Return the safety editor, robot model publisher, and RViz launch."""
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("tego_description"), "rviz", "display.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile_name",
                default_value="piper_profile.yaml",
                description=(
                    "Profile filename. The editable workspace config is preferred; "
                    "set TEGO_PROFILE_DIR to override it."
                ),
            ),
            DeclareLaunchArgument(
                "box_resize_step",
                default_value="0.05",
                description="Half-side increment used by the Box resize menu (m).",
            ),
            DeclareLaunchArgument(
                "plane_size",
                default_value="0.8",
                description="Displayed virtual-wall side length (m).",
            ),
            OpaqueFunction(function=_arm_safety_monitor_from_profile),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            OpaqueFunction(function=_robot_state_publisher_from_profile),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
