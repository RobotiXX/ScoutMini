import math

import pytest

from scoutmini_social_navigation.fusion import (
    equirectangular_bearing,
    normalize_angle,
    numeric_track_id,
    robust_scan_range,
    rotate_vector_by_quaternion,
    spatially_separated_indices,
    transform_point,
)


def test_equirectangular_cardinal_bearings():
    assert equirectangular_bearing(960, 1920) == pytest.approx(0.0)
    assert equirectangular_bearing(1440, 1920) == pytest.approx(math.pi / 2)
    assert equirectangular_bearing(480, 1920) == pytest.approx(-math.pi / 2)


def test_normalize_angle_wraps():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)


def test_scan_range_prefers_foreground_cluster():
    ranges = [8.0] * 101
    ranges[48:53] = [2.0, 2.1, 2.0, 2.2, 2.1]
    result = robust_scan_range(
        ranges,
        -0.5,
        0.01,
        0.0,
        0.05,
        0.3,
        10.0,
    )
    assert result == pytest.approx(2.05, abs=0.11)


def test_scan_range_rejects_insufficient_samples():
    result = robust_scan_range(
        [float('inf'), 2.0, float('nan')],
        -0.1,
        0.1,
        0.0,
        0.2,
        0.3,
        10.0,
        min_samples=2,
    )
    assert result is None


def test_quaternion_rotation_and_translation():
    half = math.sqrt(0.5)
    rotated = rotate_vector_by_quaternion((1.0, 0.0, 0.0), (0.0, 0.0, half, half))
    assert rotated == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)
    point = transform_point((1.0, 0.0, 0.0), (2.0, 3.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert point == pytest.approx((3.0, 3.0, 0.0))


def test_numeric_track_id_matches_social_force_contract():
    assert numeric_track_id('person_000042') == 42


def test_spatial_separation_keeps_highest_confidence_candidate():
    candidates = [
        (0.0, 0.0, 0.7),
        (0.1, 0.1, 0.9),
        (1.0, 0.0, 0.6),
    ]

    assert spatially_separated_indices(candidates, 0.3) == [1, 2]
