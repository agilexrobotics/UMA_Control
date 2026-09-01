"""Pure helpers for live-feedback teleoperation state."""

import threading
from typing import Callable, Sequence

import numpy as np


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
