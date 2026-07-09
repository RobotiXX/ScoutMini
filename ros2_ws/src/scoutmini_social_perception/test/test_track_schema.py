from builtin_interfaces.msg import Time

from scoutmini_social_perception.track_schema import (
    bearing_from_equirectangular,
    make_image_markers,
    normalize_angle,
)
from scoutmini_social_perception.people_frame_transform_node import transform_person


def test_bearing_from_equirectangular_center_is_zero():
    assert abs(bearing_from_equirectangular(960, 1920)) < 1e-9


def test_bearing_from_equirectangular_edges_wrap():
    left = bearing_from_equirectangular(0, 1920)
    right = bearing_from_equirectangular(1920, 1920)
    assert abs(abs(left) - 3.141592653589793) < 1e-6
    assert abs(abs(right) - 3.141592653589793) < 1e-6


def test_normalize_angle():
    assert abs(normalize_angle(0.0)) < 1e-9
    assert abs(normalize_angle(6.283185307179586)) < 1e-9


def test_make_image_markers_accepts_none_track_id():
    stamp = Time()
    markers = make_image_markers(
        stamp,
        'camera_360',
        [{'track_id': None, 'label': 'person', 'confidence': 0.9}],
        'test',
    )
    assert len(markers.markers) == 1
    assert markers.markers[0].id == 0


def test_transform_person_rotates_and_translates_position():
    from geometry_msgs.msg import TransformStamped

    transform = TransformStamped()
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = 20.0
    transform.transform.rotation.z = 0.7071067811865475
    transform.transform.rotation.w = 0.7071067811865476

    person = {'track_id': 1, 'x': 1.0, 'y': 0.0, 'z': 0.0, 'vx': 1.0, 'vy': 0.0}
    out = transform_person(person, transform)

    assert abs(out['x'] - 10.0) < 1e-6
    assert abs(out['y'] - 21.0) < 1e-6
    assert abs(out['vx']) < 1e-6
    assert abs(out['vy'] - 1.0) < 1e-6
