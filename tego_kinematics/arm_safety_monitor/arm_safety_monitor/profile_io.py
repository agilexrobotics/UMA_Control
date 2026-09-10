"""Profile YAML editing helpers for the RViz safety-space editor."""

from __future__ import annotations

import math
import copy
import os
import re
import stat
import tempfile
from pathlib import Path
from typing import Any, Iterable, Mapping, MutableMapping, Sequence

import yaml


class ProfileValidationError(ValueError):
    """Raised when a safety profile cannot be used or saved safely."""


def _vector(value: Sequence[float], name: str) -> list[float]:
    if len(value) != 3:
        raise ProfileValidationError(f"{name} 必须是 3 个元素")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ProfileValidationError(f"{name} 必须只包含数字") from exc
    if not all(math.isfinite(item) for item in result):
        raise ProfileValidationError(f"{name} 必须只包含有限数字")
    return result


def _required_mapping(document: MutableMapping[str, Any], key: str) -> dict[str, Any]:
    value = document.setdefault(key, {})
    if not isinstance(value, dict):
        raise ProfileValidationError(f"{key} 必须是 mapping")
    return value


def _normalize_normal(normal: Sequence[float]) -> list[float]:
    result = _vector(normal, "normal")
    norm = math.sqrt(sum(item * item for item in result))
    if norm <= 0.0:
        raise ProfileValidationError("normal 必须是非零向量")
    return [item / norm for item in result]


def load_profile_document(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Load a profile while retaining fields unknown to the editor."""
    profile_path = Path(path)
    try:
        with profile_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as exc:
        raise ProfileValidationError(f"无法读取 profile: {profile_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ProfileValidationError(f"profile YAML 解析失败: {exc}") from exc
    if not isinstance(document, dict):
        raise ProfileValidationError("profile 顶层必须是 mapping")
    return document


def safety_section(document: MutableMapping[str, Any]) -> dict[str, Any]:
    """Return the mutable safety mapping, creating it when necessary."""
    return _required_mapping(document, "safety")


def _workspace_is_sided(workspace):
    if not isinstance(workspace, dict):
        raise ProfileValidationError("safety.workspace 必须是 mapping")
    keys = {"left", "right", "default"}
    if workspace and not {"frame", "type", "min", "max", "margin", "protected_frames"}.intersection(workspace):
        if set(workspace) - keys:
            raise ProfileValidationError("workspace 分侧键必须为 left/right/default")
        return True
    if keys.intersection(workspace):
        raise ProfileValidationError("workspace 不能混合共享 Box 和左右配置")
    return False


def workspace_for(document, side="", *, editable=False):
    """Select a box; copy a fallback before editing so the other side is intact."""
    if side not in ("", "left", "right"):
        raise ProfileValidationError("side 必须为 left/right")
    raw = document.get("safety", {}).get("workspace")
    if raw is None:
        return None
    sided = _workspace_is_sided(raw)
    if sided:
        if not side:
            raise ProfileValidationError("分侧 workspace 需要指定 side:=left/right")
        key = side if side in raw else "default"
        if key not in raw:
            raise ProfileValidationError(f"workspace 缺少 {side} 或 default")
        box = raw[key]
    else:
        box = raw
    if editable and side:
        if not sided:
            raw = {"default": raw}
            safety_section(document)["workspace"] = raw
        # YAML aliases can share an explicit side as well as the default.
        raw[side] = copy.deepcopy(box)
        box = raw[side]
    return box or None


def _store_workspace(document, box, side):
    if side not in ("", "left", "right"):
        raise ProfileValidationError("side 必须为 left/right")
    safety = safety_section(document)
    raw = safety.get("workspace")
    sided = raw is not None and _workspace_is_sided(raw)
    if not side:
        if sided:
            raise ProfileValidationError("分侧 workspace 需要指定 side:=left/right")
        safety["workspace"] = box
    else:
        if not sided:
            raw = {"default": raw or {}}
            safety["workspace"] = raw
        raw[side] = box


def set_workspace_box(
    document: MutableMapping[str, Any],
    *,
    side: str = "",
    frame: str,
    center: Sequence[float],
    size: Sequence[float],
    margin: float,
    protected_frames: Iterable[str],
) -> None:
    """Create or replace the axis-aligned workspace box in a profile."""
    if not frame:
        raise ProfileValidationError("workspace.frame 不能为空")
    center_values = _vector(center, "workspace.center")
    size_values = _vector(size, "workspace.size")
    if any(item <= 0.0 for item in size_values):
        raise ProfileValidationError("workspace.size 必须全部大于 0")
    try:
        margin_value = float(margin)
    except (TypeError, ValueError) as exc:
        raise ProfileValidationError("workspace.margin 必须是数字") from exc
    if not math.isfinite(margin_value) or margin_value < 0.0:
        raise ProfileValidationError("workspace.margin 必须是非负有限数字")
    protected = [str(name) for name in protected_frames if str(name)]
    if not protected:
        raise ProfileValidationError("workspace.protected_frames 不能为空")

    half = [item / 2.0 for item in size_values]
    box_min = [round(center_values[i] - half[i], 12) for i in range(3)]
    box_max = [round(center_values[i] + half[i], 12) for i in range(3)]
    box = {
        "frame": frame,
        "type": "box",
        "min": box_min,
        "max": box_max,
        "margin": margin_value,
        "protected_frames": protected,
    }
    _store_workspace(document, box, side)


def remove_workspace_box(document: MutableMapping[str, Any], side: str = "") -> None:
    """An empty side override disables its box without re-enabling a default."""
    _store_workspace(document, {}, side)
    if not side:
        safety_section(document).pop("workspace", None)


def set_virtual_wall(
    document: MutableMapping[str, Any],
    *,
    name: str,
    frame: str,
    point: Sequence[float],
    normal: Sequence[float],
    keep_side: str,
    margin: float,
    protected_frames: Iterable[str],
) -> None:
    """Create or replace a plane wall in a profile."""
    if not name:
        raise ProfileValidationError("wall.name 不能为空")
    if not frame:
        raise ProfileValidationError("wall.frame 不能为空")
    if keep_side not in ("positive", "negative"):
        raise ProfileValidationError("wall.keep_side 必须为 positive 或 negative")
    point_values = _vector(point, "wall.point")
    normal_values = _normalize_normal(normal)
    try:
        margin_value = float(margin)
    except (TypeError, ValueError) as exc:
        raise ProfileValidationError("wall.margin 必须是数字") from exc
    if not math.isfinite(margin_value) or margin_value < 0.0:
        raise ProfileValidationError("wall.margin 必须是非负有限数字")
    protected = [str(item) for item in protected_frames if str(item)]
    if not protected:
        raise ProfileValidationError("wall.protected_frames 不能为空")

    safety = safety_section(document)
    walls = safety.setdefault("virtual_walls", [])
    if not isinstance(walls, list):
        raise ProfileValidationError("safety.virtual_walls 必须是 list")
    wall = {
        "name": name,
        "frame": frame,
        "type": "plane",
        "point": point_values,
        "normal": normal_values,
        "keep_side": keep_side,
        "margin": margin_value,
        "protected_frames": protected,
    }
    for index, existing in enumerate(walls):
        if isinstance(existing, dict) and existing.get("name") == name:
            walls[index] = wall
            break
    else:
        walls.append(wall)


def remove_virtual_wall(document: MutableMapping[str, Any], name: str) -> None:
    """Remove a named plane wall from a profile."""
    safety = safety_section(document)
    walls = safety.get("virtual_walls", [])
    if not isinstance(walls, list):
        raise ProfileValidationError("safety.virtual_walls 必须是 list")
    safety["virtual_walls"] = [
        wall for wall in walls if not isinstance(wall, dict) or wall.get("name") != name
    ]


def flip_virtual_wall_normal(
    document: MutableMapping[str, Any], name: str
) -> list[float]:
    """Flip and return the unit normal of a named virtual wall."""
    safety = safety_section(document)
    walls = safety.get("virtual_walls", [])
    if not isinstance(walls, list):
        raise ProfileValidationError("safety.virtual_walls 必须是 list")
    for wall in walls:
        if not isinstance(wall, dict) or wall.get("name") != name:
            continue
        normal = _normalize_normal(wall.get("normal", []))
        wall["normal"] = [-item for item in normal]
        return wall["normal"]
    raise ProfileValidationError(f"virtual wall 不存在: {name}")


def validate_safety_document(document: Mapping[str, Any]) -> None:
    """Validate geometry-independent safety fields before writing YAML."""
    if not isinstance(document, Mapping):
        raise ProfileValidationError("profile 顶层必须是 mapping")
    safety = document.get("safety", {})
    if not isinstance(safety, Mapping):
        raise ProfileValidationError("safety 必须是 mapping")

    raw_workspace = safety.get("workspace")
    if raw_workspace is not None:
        sided = _workspace_is_sided(raw_workspace)
        boxes = raw_workspace.values() if sided else [raw_workspace]
        for workspace in boxes:
            if not isinstance(workspace, dict):
                raise ProfileValidationError("workspace Box 必须为 mapping；用 {} 关闭")
            if not workspace:
                continue
            if workspace.get("type", "box") != "box":
                raise ProfileValidationError("safety.workspace.type 必须为 box")
            box_min = _vector(workspace.get("min", []), "workspace.min")
            box_max = _vector(workspace.get("max", []), "workspace.max")
            for index, (low, high) in enumerate(zip(box_min, box_max)):
                if low >= high:
                    raise ProfileValidationError(
                        f"workspace.min[{index}] 必须小于 max[{index}]"
                    )
            margin = float(workspace.get("margin", 0.0))
            if not math.isfinite(margin) or margin < 0.0:
                raise ProfileValidationError("workspace.margin 必须是非负有限数字")
            if not workspace.get("frame") or any(low + margin >= high - margin for low, high in zip(box_min, box_max)):
                raise ProfileValidationError("workspace 需要 frame 且 margin 后仍有非空空间")
            if not workspace.get("protected_frames"):
                raise ProfileValidationError("workspace.protected_frames 不能为空")

    walls = safety.get("virtual_walls", [])
    if not isinstance(walls, list):
        raise ProfileValidationError("safety.virtual_walls 必须是 list")
    names: set[str] = set()
    for wall in walls:
        if not isinstance(wall, Mapping):
            raise ProfileValidationError("每个 virtual wall 必须是 mapping")
        name = str(wall.get("name", ""))
        if not name:
            raise ProfileValidationError("wall.name 不能为空")
        if name in names:
            raise ProfileValidationError(f"wall.name 重复: {name}")
        names.add(name)
        if wall.get("type", "plane") != "plane":
            raise ProfileValidationError(f"wall {name}: type 必须为 plane")
        _vector(wall.get("point", []), f"wall {name}.point")
        _normalize_normal(wall.get("normal", []))
        if wall.get("keep_side", "positive") not in ("positive", "negative"):
            raise ProfileValidationError(f"wall {name}.keep_side 无效")
        margin = float(wall.get("margin", 0.0))
        if not math.isfinite(margin) or margin < 0.0:
            raise ProfileValidationError(f"wall {name}.margin 必须是非负有限数字")
        if not wall.get("protected_frames"):
            raise ProfileValidationError(f"wall {name}.protected_frames 不能为空")


def atomic_save_profile(
    path: str | os.PathLike[str], document: Mapping[str, Any]
) -> None:
    """Incrementally save safety data without creating a backup file."""
    validate_safety_document(document)
    profile_path = Path(path)
    profile_path.parent.mkdir(parents=True, exist_ok=True)
    original_text = profile_path.read_text(encoding="utf-8") if profile_path.exists() else ""
    original_mode = (
        stat.S_IMODE(profile_path.stat().st_mode) if profile_path.exists() else None
    )
    content = _update_safety_section(original_text, document.get("safety", {}))

    fd, temp_name = tempfile.mkstemp(
        prefix=f".{profile_path.name}.", suffix=".tmp", dir=str(profile_path.parent)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        if original_mode is not None:
            os.chmod(temp_name, original_mode)
        os.replace(temp_name, profile_path)
    except Exception:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise


_TOP_LEVEL_KEY = re.compile(r"^([A-Za-z_][A-Za-z0-9_-]*):(?:[ \t].*)?(?:\r?\n)?$")


def _update_safety_section(original_text: str, safety: Any) -> str:
    """
    Replace the safety block and move it to the end of a YAML mapping.

    PyYAML does not retain comments.  The profile is therefore treated as text:
    all sections except ``safety`` are kept byte-for-byte, while the editable
    safety mapping is serialized and appended after the preserved sections.
    Comments found inside the old safety block are retained immediately above
    the new block.
    """
    lines = original_text.splitlines(keepends=True)
    safety_start = None
    safety_end = len(lines)
    for index, line in enumerate(lines):
        match = _TOP_LEVEL_KEY.match(line)
        if match and match.group(1) == "safety":
            safety_start = index
            break
    if safety_start is not None:
        for index in range(safety_start + 1, len(lines)):
            if _TOP_LEVEL_KEY.match(lines[index]):
                safety_end = index
                break
        old_safety_lines = lines[safety_start:safety_end]
        preserved_text = "".join(lines[:safety_start] + lines[safety_end:])
        comments = _extract_comments(old_safety_lines)
    else:
        preserved_text = original_text
        comments = []

    safety_yaml = yaml.safe_dump(
        {"safety": safety},
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
    )
    if comments:
        safety_yaml = "\n".join(comments) + "\n" + safety_yaml

    preserved_text = preserved_text.rstrip()
    if preserved_text:
        return preserved_text + "\n\n" + safety_yaml
    return safety_yaml


def _extract_comments(lines: Iterable[str]) -> list[str]:
    """Return comment text from a replaced section without YAML indentation."""
    comments: list[str] = []
    for line in lines:
        marker = line.find("#")
        if marker < 0:
            continue
        comment = line[marker:].strip()
        if comment and comment not in comments:
            comments.append(comment)
    return comments
