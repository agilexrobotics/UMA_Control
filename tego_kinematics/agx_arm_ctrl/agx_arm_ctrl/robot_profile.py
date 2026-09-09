"""Robot profile: YAML loading, validation and joint name mapping.

ROS-free; JointState is duck-typed (needs .name/.position lists).
No sign/offset: pure name mapping; direction/zero live in the hardware layer.
"""

import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union

import numpy as np
import yaml


INSTALLATION_POSITIONS = ("horizontal", "left", "right")
INSTALLATION_POSITION_KEYS = ("default", "left", "right")
EFFECTOR_TYPES = ("none", "agx_gripper", "revo2")


def validate_installation_pos(value: Any, context: str = "installation_pos") -> str:
    """Validate and return one physical arm installation orientation."""
    if not isinstance(value, str) or value not in INSTALLATION_POSITIONS:
        allowed = ", ".join(INSTALLATION_POSITIONS)
        raise ValueError(f"{context} 必须为 {allowed}, got {value!r}")
    return value


def validate_effector_type(value: Any) -> str:
    """Validate and return one driver end-effector type."""
    if not isinstance(value, str) or value not in EFFECTOR_TYPES:
        allowed = ", ".join(EFFECTOR_TYPES)
        raise ValueError(f"robot.effector_type 必须为 {allowed}, got {value!r}")
    return value


@dataclass
class JointMapping:
    urdf_name: str
    feedback_name: str
    command_name: str


@dataclass
class SafetyFrame:
    """Runtime safety frame (robot.safety_frames), created by addFrame.

    auto=True: origin read from the URDF joint origin; no xyz/rpy needed.
    auto=False: custom placement via parent_joint + xyz/rpy.
    """

    parent_joint: str
    xyz: List[float] = field(default_factory=list)
    rpy: List[float] = field(default_factory=list)
    auto: bool = False


@dataclass
class EEOffset:
    parent_joint: str
    xyz: List[float]
    rpy: List[float]


@dataclass
class WorkspaceBox:
    """Box workspace (safety.workspace): points stay inside, with margin."""

    frame: str
    min: List[float]
    max: List[float]
    margin: float = 0.0
    protected_frames: List[str] = field(default_factory=list)


@dataclass
class PlaneWall:
    """Plane wall (safety.virtual_walls): n·(p - point) by keep_side.

    keep_side=positive: n·(p - point) >= margin;
    negative: n·(p - point) <= -margin.
    normal is normalized at load; margin is the normal distance.
    """

    name: str
    frame: str
    point: List[float]
    normal: List[float]
    keep_side: str = "positive"
    margin: float = 0.0
    protected_frames: List[str] = field(default_factory=list)


@dataclass
class RobotProfile:
    """One arm profile. model/nq/urdf_path_abs filled by validate()."""

    name: str
    urdf_path: str
    base_frame: str
    joints: List[JointMapping] = field(default_factory=list)
    locked_joints: List[str] = field(default_factory=list)
    safety_frames: Dict[str, SafetyFrame] = field(default_factory=dict)
    ee_frame: Optional[str] = None
    ee_offset: Optional[EEOffset] = None
    workspace_box: Optional[WorkspaceBox] = None
    virtual_walls: List[PlaneWall] = field(default_factory=list)
    collision_pairs: List[List[int]] = field(default_factory=list)
    arm_home_state: Dict[str, List[float]] = field(default_factory=dict)
    # 手柄桥接参数(tego_ros 节点),如 zoom_factor;由手柄 launch 读取
    handle: Dict[str, Any] = field(default_factory=dict)
    # 通用物理安装朝向；可按通信 side 配置。支持的驱动还会用它配置重力补偿。
    installation_pos: Union[str, Dict[str, str]] = "horizontal"
    # None 保留各 launch 的旧默认值；none 显式关闭末端执行器。
    effector_type: Optional[str] = None
    enable_hard_delta_q_constraint: bool = True
    max_delta_q_rad: float = 0.15
    enable_psi_constraint: bool = False
    psi_weight: float = 30.0
    psi_eps: float = 0.05
    enable_joint_reference_penalty: bool = False
    joint_reference_weight: float = 0.0
    joint_reference_mask: Optional[List[float]] = None
    enable_elbow_side_penalty: bool = False
    elbow_side_penalty_weight: float = 0.0
    elbow_side_penalty_sharpness: float = 15.0
    shoulder_frame: Optional[str] = None
    elbow_frame: Optional[str] = None
    wrist_frame: Optional[str] = None
    singularity_weight: float = 0.02

    # Cache filled by validate()
    urdf_path_abs: str = ""
    package_dirs: List[str] = field(default_factory=list)
    model: Optional[object] = None
    nq: int = 0

    def feedback_to_q(self, msg):
        """Build URDF-order q from a validated JointState."""
        if len(msg.name) != len(msg.position):
            raise ValueError("反馈 name/position 长度不一致")
        if len(set(msg.name)) != len(msg.name):
            raise ValueError("反馈关节名重复")
        name_to_pos = dict(zip(msg.name, msg.position))
        missing = [
            m.feedback_name for m in self.joints if m.feedback_name not in name_to_pos
        ]
        if missing:
            raise ValueError(f"反馈缺少关节: {missing}")
        try:
            q = np.array(
                [name_to_pos[m.feedback_name] for m in self.joints], dtype=float
            )
        except (TypeError, ValueError) as e:
            raise ValueError("反馈关节角必须为数字") from e
        if not np.isfinite(q).all():
            raise ValueError("反馈关节角必须只包含有限数字")
        return q

    def q_to_command(self, q):
        """IK solution (URDF order) -> (command names, positions)."""
        try:
            q = np.asarray(q, dtype=float).reshape(-1)
        except (TypeError, ValueError) as e:
            raise ValueError("IK 关节角必须为数字") from e
        if q.size != self.nq:
            raise ValueError(f"q 维度 {q.size} != profile nq {self.nq}")
        if not np.isfinite(q).all():
            raise ValueError("IK 关节角必须只包含有限数字")
        return [m.command_name for m in self.joints], q.tolist()

    def installation_pos_for(self, side: str) -> str:
        """Resolve the physical installation orientation for one routed arm."""
        if side not in ("left", "right"):
            raise ValueError(f"unsupported arm side: {side}")
        if isinstance(self.installation_pos, str):
            return validate_installation_pos(
                self.installation_pos, "robot.installation_pos"
            )
        value = self.installation_pos.get(side)
        if value is None:
            value = self.installation_pos.get("default")
        if value is None:
            raise ValueError(
                f"robot.installation_pos 缺少键: {side}(或 default)"
            )
        return validate_installation_pos(
            value, f"robot.installation_pos.{side}"
        )


def _resolve_urdf_path(urdf_path: str):
    if urdf_path.startswith("package://"):
        pkg, _, rel = urdf_path[len("package://") :].partition("/")
        try:
            from ament_index_python.packages import get_package_share_directory
        except ImportError as e:
            raise ValueError("package:// 路径需要 ament_index_python(ROS 环境)") from e
        share = get_package_share_directory(pkg)
        return os.path.join(share, rel), [os.path.dirname(share)]
    return urdf_path, []


def _parse_ik(raw: dict, profile: RobotProfile) -> None:
    ik_fields = [
        "enable_hard_delta_q_constraint",
        "max_delta_q_rad",
        "enable_psi_constraint",
        "psi_weight",
        "psi_eps",
        "enable_joint_reference_penalty",
        "joint_reference_weight",
        "joint_reference_mask",
        "enable_elbow_side_penalty",
        "elbow_side_penalty_weight",
        "elbow_side_penalty_sharpness",
        "shoulder_frame",
        "elbow_frame",
        "wrist_frame",
        "singularity_weight",
    ]
    for key in ik_fields:
        if key in raw:
            setattr(profile, key, raw[key])


def _parse_installation_pos(robot_raw: dict) -> Union[str, Dict[str, str]]:
    if "installation_pos" not in robot_raw:
        return "horizontal"

    raw = robot_raw["installation_pos"]
    if isinstance(raw, str):
        return validate_installation_pos(raw, "robot.installation_pos")
    if not isinstance(raw, dict) or not raw:
        raise ValueError(
            "robot.installation_pos 必须为 horizontal/left/right 或非空映射"
        )

    parsed = {}
    for key, value in raw.items():
        if not isinstance(key, str) or key not in INSTALLATION_POSITION_KEYS:
            allowed = ", ".join(INSTALLATION_POSITION_KEYS)
            raise ValueError(
                f"robot.installation_pos 映射键必须为 {allowed}, got {key!r}"
            )
        parsed[key] = validate_installation_pos(
            value, f"robot.installation_pos.{key}"
        )
    return parsed


def _frame_exists(model, name: str) -> bool:
    # NOTE: pinocchio 3.2 getFrameId does NOT raise on unknown names
    # (it returns an out-of-range id); check against the frames list.
    return name in {f.name for f in model.frames}


def _derive_joints(model, locked_joints: List[str]) -> List[JointMapping]:
    """Derive arm joints from the URDF: revolute, URDF order, minus locked."""

    locked = set(locked_joints)
    joints = []
    for i in range(1, model.njoints):
        name = model.names[i]
        if name in locked:
            continue
        shortname = model.joints[i].shortname()
        if shortname.startswith("JointModelR"):  # revolute
            joints.append(JointMapping(name, name, name))
    return joints


def _known_frame(profile: "RobotProfile", model, name: str) -> bool:
    """Frame exists in URDF or is created at runtime (ee / safety frames)."""
    if name == "ee" and profile.ee_offset is not None and profile.ee_frame is None:
        return True
    if name in profile.safety_frames:
        return True
    return _frame_exists(model, name)


def validate(profile: RobotProfile) -> None:
    """Validate profile; fill urdf_path_abs/package_dirs/model/joints/nq."""
    try:
        import pinocchio as pin
    except ImportError as e:
        raise ValueError("profile URDF 校验需要 pinocchio 运行环境") from e

    profile.urdf_path_abs, profile.package_dirs = _resolve_urdf_path(profile.urdf_path)
    if not os.path.isfile(profile.urdf_path_abs):
        raise ValueError(f"URDF 不存在: {profile.urdf_path_abs}")
    try:
        model = pin.buildModelFromUrdf(profile.urdf_path_abs)
    except Exception as e:
        raise ValueError(f"URDF 加载失败: {e}") from e
    profile.model = model

    # Joints are derived from the URDF (revolute minus locked), not from YAML.
    profile.joints = _derive_joints(model, profile.locked_joints)
    if not profile.joints:
        raise ValueError("URDF 中没有可用的 revolute 关节(可能 locked_joints 配错)")
    urdf_names = [j.urdf_name for j in profile.joints]

    model_joint_names = set(model.names)
    for name in profile.locked_joints:
        if name not in model_joint_names:
            raise ValueError(f"locked_joints 关节不存在: {name}")

    if not _frame_exists(model, profile.base_frame):
        raise ValueError(f"base_frame 不存在: {profile.base_frame}")

    if profile.ee_frame is not None:
        if not _frame_exists(model, profile.ee_frame):
            raise ValueError(f"ee_frame 不存在: {profile.ee_frame}")
    elif profile.ee_offset is not None:
        off = profile.ee_offset
        if off.parent_joint not in model_joint_names:
            raise ValueError(f"ee_offset.parent_joint 不存在: {off.parent_joint}")
        if len(off.xyz) != 3 or len(off.rpy) != 3:
            raise ValueError("ee_offset.xyz/rpy 必须为 3 元素")
    else:
        raise ValueError("必须提供 ee_frame 或 ee_offset 之一")

    for name, sf in profile.safety_frames.items():
        if sf.parent_joint not in model_joint_names:
            raise ValueError(
                f"safety_frames.{name}.parent_joint 不存在: {sf.parent_joint}"
            )
        if not sf.auto and (len(sf.xyz) != 3 or len(sf.rpy) != 3):
            raise ValueError(f"safety_frames.{name}.xyz/rpy 必须为 3 元素")
        if name == "ee" or _frame_exists(model, name):
            raise ValueError(f"safety_frames.{name} 与 URDF/ee 帧重名")

    nq = model.nq - len(profile.locked_joints)  # single-DOF joints assumed
    max_link_idx = model.njoints - 1  # link/geom index space (world=0)
    for i, j in profile.collision_pairs:
        if not (0 <= i <= max_link_idx and 0 <= j <= max_link_idx):
            raise ValueError(
                f"collision_pairs 索引越界: ({i},{j}), 有效范围 0..{max_link_idx}"
            )

    if profile.workspace_box is not None:
        box = profile.workspace_box
        if len(box.min) != 3 or len(box.max) != 3:
            raise ValueError("safety.workspace.min/max 必须为 3 元素")
        if box.margin < 0.0:
            raise ValueError("safety.workspace.margin 必须 >= 0")
        if not box.protected_frames:
            raise ValueError("safety.workspace.protected_frames 不能为空")
        if not _frame_exists(model, box.frame):
            raise ValueError(f"safety.workspace.frame 不存在: {box.frame}")
        for i in range(3):
            if box.min[i] >= box.max[i]:
                raise ValueError(f"safety.workspace.min[{i}] 必须小于 max[{i}]")
        for name in box.protected_frames:
            if not _known_frame(profile, model, name):
                raise ValueError(f"safety.workspace.protected_frames 不存在: {name}")

    wall_names = [w.name for w in profile.virtual_walls]
    if len(set(wall_names)) != len(wall_names):
        raise ValueError("safety.virtual_walls.name 重复")
    for w in profile.virtual_walls:
        if len(w.point) != 3 or len(w.normal) != 3:
            raise ValueError(f"wall {w.name}: point/normal 必须为 3 元素")
        norm = float(np.linalg.norm(w.normal))
        if norm <= 0.0:
            raise ValueError(f"wall {w.name}: normal 必须非零")
        w.normal = (np.array(w.normal, dtype=float) / norm).tolist()
        if w.margin < 0.0:
            raise ValueError(f"wall {w.name}: margin 必须 >= 0")
        if w.keep_side not in ("positive", "negative"):
            raise ValueError(f"wall {w.name}: keep_side 必须为 positive/negative")
        if not w.protected_frames:
            raise ValueError(f"wall {w.name}: protected_frames 不能为空")
        if not _frame_exists(model, w.frame):
            raise ValueError(f"wall {w.name}: frame 不存在: {w.frame}")
        for name in w.protected_frames:
            if not _known_frame(profile, model, name):
                raise ValueError(f"wall {w.name}: protected_frames 不存在: {name}")

    if profile.enable_psi_constraint or profile.enable_elbow_side_penalty:
        for name in (profile.shoulder_frame, profile.elbow_frame, profile.wrist_frame):
            if not name:
                raise ValueError("启用 psi/elbow 时必须提供 shoulder/elbow/wrist_frame")
            if not _known_frame(profile, model, name):
                raise ValueError(f"frame 不存在: {name}")

    profile.nq = nq


def load_profile(path: str, validate_robot: bool = True) -> RobotProfile:
    """
    Load a profile from YAML, optionally validating its URDF/IK model.
    """
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ValueError("profile 必须是 YAML 映射")
    robot_raw = raw.get("robot")
    if not isinstance(robot_raw, dict):
        raise ValueError("profile 缺少 robot 节")

    safety_frames_raw = (
        (raw.get("safety") or {}).get("safety_frames")
        or robot_raw.get("safety_frames")
        or {}
    )
    safety_frames = {}
    for name, sf_raw in safety_frames_raw.items():
        if isinstance(sf_raw, str):
            # Short form: frame at the joint origin, read from URDF at runtime.
            safety_frames[name] = SafetyFrame(parent_joint=sf_raw, auto=True)
        else:
            safety_frames[name] = SafetyFrame(
                parent_joint=sf_raw.get("parent_joint", ""),
                xyz=list(sf_raw.get("xyz", [])),
                rpy=list(sf_raw.get("rpy", [])),
            )

    ee_offset = None
    if robot_raw.get("ee_offset") is not None:
        off = robot_raw["ee_offset"]
        ee_offset = EEOffset(
            off.get("parent_joint", ""),
            list(off.get("xyz", [])),
            list(off.get("rpy", [])),
        )

    workspace_box = None
    safety_raw = raw.get("safety") or {}
    ws_raw = safety_raw.get("workspace") or {}
    if ws_raw:
        ws_type = ws_raw.get("type", "box")
        if ws_type != "box":
            raise ValueError(f"safety.workspace.type 必须为 box, got {ws_type}")
        workspace_box = WorkspaceBox(
            frame=ws_raw.get("frame", ""),
            min=list(ws_raw.get("min", [])),
            max=list(ws_raw.get("max", [])),
            margin=float(ws_raw.get("margin", 0.0)),
            protected_frames=list(ws_raw.get("protected_frames") or []),
        )

    virtual_walls = []
    for w_raw in safety_raw.get("virtual_walls") or []:
        w_type = w_raw.get("type", "plane")
        if w_type != "plane":
            raise ValueError(
                f"wall {w_raw.get('name', '')}: type 必须为 plane, got {w_type}"
            )
        normal = np.array(list(w_raw.get("normal", [])), dtype=float)
        norm = float(np.linalg.norm(normal))
        if norm > 0.0:
            normal = (normal / norm).tolist()
        virtual_walls.append(
            PlaneWall(
                name=w_raw.get("name", ""),
                frame=w_raw.get("frame", ""),
                point=list(w_raw.get("point", [])),
                normal=normal,
                keep_side=w_raw.get("keep_side", "positive"),
                margin=float(w_raw.get("margin", 0.0)),
                protected_frames=list(w_raw.get("protected_frames") or []),
            )
        )

    raw_home_state = robot_raw.get("arm_home_state") or {}
    if isinstance(raw_home_state, list):
        raw_home_state = {"default": raw_home_state}
    if not isinstance(raw_home_state, dict):
        raise ValueError("robot.arm_home_state 必须是列表或映射")

    arm_home_state = {}
    for key, values in raw_home_state.items():
        if not isinstance(values, (list, tuple)) or not values:
            raise ValueError(f"robot.arm_home_state.{key} 必须是非空列表")
        try:
            parsed_values = [float(value) for value in values]
        except (TypeError, ValueError) as e:
            raise ValueError(f"robot.arm_home_state.{key} 必须只包含数字") from e
        if not all(math.isfinite(value) for value in parsed_values):
            raise ValueError(f"robot.arm_home_state.{key} 必须只包含有限数字")
        arm_home_state[str(key)] = parsed_values

    profile = RobotProfile(
        name=robot_raw.get("name", ""),
        urdf_path=robot_raw.get("urdf_path", ""),
        base_frame=robot_raw.get("base_frame", ""),
        locked_joints=list(robot_raw.get("locked_joints") or []),
        arm_home_state=arm_home_state,
        safety_frames=safety_frames,
        ee_frame=robot_raw.get("ee_frame"),
        ee_offset=ee_offset,
        workspace_box=workspace_box,
        virtual_walls=virtual_walls,
        collision_pairs=[list(p) for p in (robot_raw.get("collision_pairs") or [])],
        handle=raw.get("handle") or {},
        installation_pos=_parse_installation_pos(robot_raw),
        effector_type=(
            validate_effector_type(robot_raw["effector_type"])
            if "effector_type" in robot_raw
            else None
        ),
    )
    _parse_ik(raw.get("ik") or {}, profile)
    _validate_home_state_lengths(profile)
    if validate_robot:
        validate(profile)
    return profile


def _validate_home_state_lengths(profile: RobotProfile) -> None:
    """All arm_home_state entries must share one length (hardware channel count)."""
    lengths = {len(values) for values in profile.arm_home_state.values()}
    if len(lengths) > 1:
        raise ValueError(f"robot.arm_home_state 各键长度不一致: {lengths}")


def format_arm_home_state(profile: RobotProfile, key: str) -> str:
    """Return a validated profile home target in the ROS parameter format.

    ``key`` wins if present; otherwise falls back to ``default``.
    """
    values = profile.arm_home_state.get(key)
    if values is None:
        values = profile.arm_home_state.get("default")
    if values is None:
        raise ValueError(f"robot.arm_home_state 缺少键: {key}(或 default)")
    if not isinstance(values, (list, tuple)) or not values:
        raise ValueError(f"robot.arm_home_state.{key} 必须是非空列表")
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f"robot.arm_home_state.{key} 必须只包含有限数字")
    return ", ".join(format(float(value), ".17g") for value in values)
