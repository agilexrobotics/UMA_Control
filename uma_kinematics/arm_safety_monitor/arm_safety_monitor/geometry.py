"""Small geometry helpers shared by the safety editor and its tests."""

from __future__ import annotations

import math
from typing import Sequence


def _unit(vector: Sequence[float]) -> tuple[float, float, float]:
    if len(vector) != 3:
        raise ValueError("vector must have three elements")
    values = tuple(float(value) for value in vector)
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError("vector must be non-zero")
    return tuple(value / norm for value in values)


def quaternion_from_normal(normal: Sequence[float]) -> tuple[float, float, float, float]:
    """Return xyzw quaternion that rotates local +Z to ``normal``."""
    nx, ny, nz = _unit(normal)
    dot = nz
    if dot < -1.0 + 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    cross_x = -ny
    cross_y = nx
    cross_z = 0.0
    scale = math.sqrt(2.0 * (1.0 + dot))
    quaternion = (cross_x / scale, cross_y / scale, cross_z / scale, scale / 2.0)
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)


def normal_from_quaternion(quaternion: Sequence[float]) -> tuple[float, float, float]:
    """Rotate local +Z by an xyzw quaternion and return the unit normal."""
    if len(quaternion) != 4:
        raise ValueError("quaternion must have four elements")
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("quaternion must be non-zero")
    x, y, z, w = (x / norm, y / norm, z / norm, w / norm)
    return (
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    )


def box_center_size(
    box_min: Sequence[float], box_max: Sequence[float]
) -> tuple[list[float], list[float]]:
    """Convert axis-aligned min/max coordinates to marker center and size."""
    if len(box_min) != 3 or len(box_max) != 3:
        raise ValueError("box min/max must have three elements")
    center = [(float(low) + float(high)) / 2.0 for low, high in zip(box_min, box_max)]
    size = [float(high) - float(low) for low, high in zip(box_min, box_max)]
    if any(value <= 0.0 for value in size):
        raise ValueError("box size must be positive")
    return center, size


def resize_box(
    box_min: Sequence[float], box_max: Sequence[float], axis: int, delta: float
) -> tuple[list[float], list[float]]:
    """Resize one box dimension around its current center."""
    center, size = box_center_size(box_min, box_max)
    if axis not in (0, 1, 2):
        raise ValueError("axis must be 0, 1 or 2")
    new_size = size[axis] + 2.0 * float(delta)
    if new_size <= 0.0:
        raise ValueError("box size must remain positive")
    size[axis] = new_size
    half = [value / 2.0 for value in size]
    return (
        [round(center[index] - half[index], 12) for index in range(3)],
        [round(center[index] + half[index], 12) for index in range(3)],
    )
