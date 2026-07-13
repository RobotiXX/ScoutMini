"""Pure geometry and scan-association helpers."""

from __future__ import annotations

import math
from statistics import median
from typing import Optional, Sequence, Tuple


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def equirectangular_bearing(
    center_x: float,
    image_width: int,
    center_x_norm: float = 0.5,
    offset_rad: float = 0.0,
) -> float:
    """Convert an equirectangular horizontal pixel to camera-frame bearing."""
    if image_width <= 0:
        raise ValueError('image_width must be positive')
    bearing = (center_x / image_width - center_x_norm) * 2.0 * math.pi
    return normalize_angle(bearing + offset_rad)


def robust_scan_range(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    target_bearing: float,
    half_width_rad: float,
    range_min: float,
    range_max: float,
    min_samples: int = 2,
) -> Optional[float]:
    """Estimate foreground range from valid scan returns in an angular window."""
    if not ranges or angle_increment == 0.0:
        return None
    valid = []
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        angle = angle_min + index * angle_increment
        if abs(normalize_angle(angle - target_bearing)) <= half_width_rad:
            valid.append(float(value))
    if len(valid) < max(1, min_samples):
        return None
    valid.sort()
    foreground_count = max(min_samples, int(math.ceil(len(valid) * 0.25)))
    return float(median(valid[:foreground_count]))


def rotate_vector_by_quaternion(
    vector: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Rotate a vector by an xyzw quaternion."""
    vx, vy, vz = vector
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def transform_point(
    point: Tuple[float, float, float],
    translation: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Apply a rigid transform to a point."""
    rotated = rotate_vector_by_quaternion(point, quaternion)
    return tuple(rotated[index] + translation[index] for index in range(3))


def numeric_track_id(track_id: str) -> int:
    """Produce the numeric ID required by the social-force planner."""
    digits = ''.join(character for character in track_id if character.isdigit())
    return int(digits) if digits else 0
