import math

from builtin_interfaces.msg import Time

from scoutmini_social_perception.adascore_people_adapter_node import (
    AdaScorePeopleAdapter,
    _person_yaw,
    should_publish_people_msg,
)
from scoutmini_social_perception.adascore_readiness_check import build_report
from scoutmini_social_perception.people_projection_node import estimate_person_range
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


def test_estimate_person_range_fixed_mode_is_clamped():
    range_m, source = estimate_person_range({}, 'fixed_distance_debug', 0.01, 1.7)

    assert abs(range_m - 0.1) < 1e-9
    assert source == 'fixed_distance_debug'


def test_estimate_person_range_from_person_height():
    range_m, source = estimate_person_range(
        {'bbox_xyxy': [100.0, 100.0, 200.0, 260.0], 'image_height': 640},
        'person_height_ground_plane',
        3.0,
        1.7,
    )

    assert abs(range_m - (1.7 / ((160.0 / 640.0) * math.pi))) < 1e-6
    assert source == 'person_height_ground_plane'


def test_person_yaw_prefers_explicit_yaw_then_velocity_then_bearing():
    explicit = {'yaw_rad': 1.25, 'vx': 1.0, 'vy': 0.0, 'bearing_rad': 0.0}
    velocity = {'vx': 0.0, 'vy': 2.0, 'bearing_rad': 0.0}

    assert abs(_person_yaw(explicit) - 1.25) < 1e-9
    assert abs(_person_yaw(velocity) - (math.pi / 2.0)) < 1e-9
    assert abs(_person_yaw({'bearing_rad': -0.4}) + 0.4) < 1e-9


def test_adascore_readiness_report_has_expected_gates():
    report = build_report()

    assert 'people_msgs' in report['ros_packages']
    assert 'adascore' in report['ros_packages']
    assert 'torch' in report['python_modules']
    assert 'tensorrt' in report['python_modules']
    assert 'yolo_tensorrt_engine' in report['model_artifacts']
    assert 'adascore_dependencies_available' in report['summary']
    assert 'cuda_pytorch_available' in report['summary']
    assert 'yolo_gpu_execution_ready' in report['summary']


class _FakePeople:
    pass


class _FakePerson:
    pass


def test_adascore_people_msg_conversion_matches_expected_contract():
    adapter = AdaScorePeopleAdapter.__new__(AdaScorePeopleAdapter)
    adapter.PeopleMsg = _FakePeople
    adapter.PersonMsg = _FakePerson

    msg = adapter._to_people_msg(
        12.25,
        'map',
        [
            {
                'track_id': 7,
                'x': 1.2,
                'y': -0.4,
                'yaw_rad': 0.75,
                'vx': 0.3,
                'vy': -0.1,
                'wz': 0.05,
                'confidence': 0.88,
                'source': 'people_frame_transform',
                'range_source': 'person_height_ground_plane',
            }
        ],
    )

    assert msg.header.stamp.sec == 12
    assert msg.header.stamp.nanosec == 250000000
    assert msg.header.frame_id == 'map'
    assert len(msg.people) == 1

    person = msg.people[0]
    assert person.name == 'person_7'
    assert abs(person.position.x - 1.2) < 1e-9
    assert abs(person.position.y + 0.4) < 1e-9
    assert abs(person.position.z - 0.75) < 1e-9
    assert abs(person.velocity.x - 0.3) < 1e-9
    assert abs(person.velocity.y + 0.1) < 1e-9
    assert abs(person.velocity.z - 0.05) < 1e-9
    assert abs(person.reliability - 0.88) < 1e-9
    assert person.tagnames == ['track_id', 'source', 'range_source']
    assert person.tags == ['7', 'people_frame_transform', 'person_height_ground_plane']


def test_people_msg_frame_gate_requires_matching_frame_when_enabled():
    assert should_publish_people_msg('map', 'map', True)
    assert not should_publish_people_msg('base_link', 'map', True)
    assert should_publish_people_msg('base_link', 'map', False)
