"""Pure helpers for live-feedback teleoperation state."""

import math
import threading
from typing import Callable, Sequence

import numpy as np

try:
    from .robot_profile import validate_installation_pos
except ImportError:  # Support direct execution beside the teleop script.
    from robot_profile import validate_installation_pos


def installation_rotation(installation_pos: str) -> np.ndarray:
    """Return the handle-world to arm-base rotation for an installation."""
    position = validate_installation_pos(installation_pos)
    # Side mappings preserve the Nero v0.1.0 home-FK translation axes.
    rotations = {
        "horizontal": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "left": [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        "right": [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    }
    return np.array(rotations[position], dtype=float)


def _rotation_matrix(value, name: str) -> np.ndarray:
    try:
        rotation = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须为 3x3 数字旋转矩阵") from exc
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError(f"{name} 必须为有限的 3x3 旋转矩阵")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"{name} 必须为正交旋转矩阵")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise ValueError(f"{name} 行列式必须为 +1")
    return rotation


def build_translation_mapping(
    installation_pos: str,
    home_rotation,
    handle_start_rotation,
):
    """Return the home correction and handle-local to arm-base rotation."""
    home = _rotation_matrix(home_rotation, "home_rotation")
    handle_start = _rotation_matrix(handle_start_rotation, "handle_start_rotation")
    correction = installation_rotation(installation_pos) @ handle_start @ home.T
    return correction, correction @ home


def build_heading_mapping(handle_start_rotation) -> np.ndarray:
    """Map the initial horizontal heading to base +X, keeping world +Z up.

    The bridge's calibrated world frame must already have a valid up reference.
    Reject a forward axis within 15 degrees of vertical: heading is unreliable.
    """
    rotation = _rotation_matrix(handle_start_rotation, "handle_start_rotation")
    forward = rotation[:, 0].copy()
    forward[2] = 0.0
    length = np.linalg.norm(forward)
    if length <= math.sin(math.radians(15)):
        raise ValueError("手柄朝向接近竖直，请将手柄放平后重新开启")
    forward /= length
    up = np.array([0.0, 0.0, 1.0])
    return np.vstack((forward, np.cross(up, forward), up))


def build_horizontal_target(arm_start, handle_start, handle_current, mapping):
    """Apply spatial pose increments through a mapping frozen at enable."""
    target = arm_start.copy()
    target[:3, 3] += mapping @ (handle_current[:3, 3] - handle_start[:3, 3])
    rotation_delta = handle_current[:3, :3] @ handle_start[:3, :3].T
    target[:3, :3] = mapping @ rotation_delta @ mapping.T @ arm_start[:3, :3]
    return target


class FeedbackSnapshot:
    """Thread-safe latest joint-feedback snapshot."""

    def __init__(self):
        self._lock = threading.Lock()
        self._positions = None
        self._received_at = None

    def update(self, positions: Sequence[float], timestamp: float) -> None:
        """Atomically replace the positions and their monotonic timestamp."""
        copied_positions = [float(value) for value in positions]
        with self._lock:
            self._positions = copied_positions
            self._received_at = float(timestamp)

    def read(self):
        """Return a consistent copy of positions and timestamp."""
        with self._lock:
            positions = (
                None if self._positions is None else list(self._positions)
            )
            return positions, self._received_at

    def is_fresh(self, now: float, timeout: float) -> bool:
        """Return whether the current snapshot is within the safety window."""
        _, received_at = self.read()
        return feedback_is_fresh(received_at, now, timeout)


def build_live_reference_matrices(
    current_joint_positions: Sequence[float],
    forward_kinematics: Callable[[Sequence[float]], np.ndarray],
) -> np.ndarray:
    """Build the teleop session anchor (start pose) from live joint feedback."""
    if current_joint_positions is None:
        raise ValueError("启动遥操前必须先收到实时关节反馈")

    current_matrix = np.asarray(
        forward_kinematics(current_joint_positions), dtype=float
    )
    if current_matrix.shape != (4, 4):
        raise ValueError(
            f"末端位姿矩阵必须为 4x4, got {current_matrix.shape}"
        )

    return current_matrix.copy()


def feedback_is_fresh(last_feedback_time: float, now: float, timeout: float) -> bool:
    """Return whether a monotonic feedback timestamp is within the safety window."""
    if last_feedback_time is None or timeout <= 0.0:
        return False
    age = now - last_feedback_time
    return 0.0 <= age <= timeout
