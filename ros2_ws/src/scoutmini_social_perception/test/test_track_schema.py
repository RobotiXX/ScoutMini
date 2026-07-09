import math
from pathlib import Path

from builtin_interfaces.msg import Time
import yaml

from scoutmini_social_perception.adascore_people_adapter_node import (
    AdaScorePeopleAdapter,
    _person_yaw,
    should_publish_people_msg,
)
from scoutmini_social_perception.adascore_preflight_check import (
    evaluate_topics,
    parse_expected_type,
    preflight_exit_code,
)
from scoutmini_social_perception.adascore_readiness_check import build_report
from scoutmini_social_perception.people_projection_node import estimate_person_range
from scoutmini_social_perception.perception_bag_validate import evaluate_report
from scoutmini_social_perception.track_schema import (
    bearing_from_equirectangular,
    make_image_markers,
    normalize_angle,
)
from scoutmini_social_perception.people_frame_transform_node import transform_person


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


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


def test_adascore_readiness_report_has_expected_gates(tmp_path):
    engine_path = tmp_path / 'custom.engine'
    engine_path.write_text('fake engine placeholder')
    report = build_report(yolo_engine_path=str(engine_path))

    assert 'people_msgs' in report['ros_packages']
    assert 'adascore' in report['ros_packages']
    assert 'torch' in report['python_modules']
    assert 'tensorrt' in report['python_modules']
    assert 'yolo_tensorrt_engine' in report['model_artifacts']
    assert report['model_artifacts']['yolo_pt']['format'] == 'pytorch'
    assert report['model_artifacts']['yolo_tensorrt_engine']['format'] == 'tensorrt'
    assert report['model_artifacts']['yolo_tensorrt_engine']['available']
    assert 'adascore_dependencies_available' in report['summary']
    assert 'cuda_pytorch_available' in report['summary']
    assert 'yolo_gpu_execution_ready' in report['summary']
    assert report['summary']['yolo_gpu_execution_ready']


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


def test_dependency_manifests_are_parseable_and_pin_expected_branches():
    people_manifest = yaml.safe_load((PACKAGE_ROOT / 'deps' / 'people_msgs_ros2.repos').read_text())
    full_manifest = yaml.safe_load((PACKAGE_ROOT / 'deps' / 'adascore_upstream_humble.repos').read_text())

    people_repo = people_manifest['repositories']['hunav/people']
    assert people_repo['url'] == 'https://github.com/wg-perception/people.git'
    assert people_repo['version'] == 'ros2'

    repos = full_manifest['repositories']
    assert repos['adascore']['url'] == 'https://github.com/maurom3197/adascore.git'
    assert repos['adascore']['version'] == 'humble'
    assert repos['hunav/people']['version'] == 'ros2'
    assert repos['hunav/hunav_sim']['version'] == 'v1.0-humble'
    assert repos['hunav/hunav_gazebo_wrapper']['version'] == 'v1.0-humble'
    assert repos['social_force_window_planner']['version'] == 'ros2'


def test_adascore_preflight_reports_missing_required_topics():
    report = evaluate_topics(['/people', '/tf'], required_topics=['/people', '/tf', '/map'])

    assert report['required_topics']['/people']
    assert report['required_topics']['/tf']
    assert not report['required_topics']['/map']
    assert report['missing_required_topics'] == ['/map']
    assert not report['summary']['required_topics_available']
    assert not report['summary']['expected_topic_types_match']
    assert not report['summary']['safe_to_start_motion']
    assert preflight_exit_code(report, fail_on_missing=False) == 0
    assert preflight_exit_code(report, fail_on_missing=True) == 2


def test_adascore_preflight_flags_motion_topics_and_normalizes_names():
    report = evaluate_topics(
        ['people', '/tf', '/map', '/cmd_vel'],
        required_topics=['/people', 'tf', 'map'],
        motion_topics=['cmd_vel'],
        topic_types={'people': ['people_msgs/msg/People']},
    )

    assert report['summary']['required_topics_available']
    assert report['summary']['expected_topic_types_match']
    assert report['summary']['motion_topics_detected']
    assert report['motion_topics_present'] == ['/cmd_vel']
    assert not report['summary']['safe_to_start_motion']
    assert preflight_exit_code(report, fail_on_missing=True) == 0


def test_perception_bag_validation_gates_counts_and_frames():
    report = {
        'topics': {
            '/people/projected': {
                'messages': 4,
                'nonempty_messages': 2,
                'frame_ids': ['adascore_bag_map'],
            },
            '/adascore/shadow/people': {
                'messages': 3,
                'nonempty_messages': 1,
                'frame_ids': ['adascore_bag_map'],
            },
        }
    }

    passed, failures = evaluate_report(
        report,
        min_messages={'/people/projected': 1, '/adascore/shadow/people': 1},
        min_nonempty={'/adascore/shadow/people': 1},
        required_frames={'/adascore/shadow/people': 'adascore_bag_map'},
    )

    assert passed
    assert failures == []


def test_perception_bag_validation_reports_failures():
    report = {
        'topics': {
            '/adascore/shadow/people': {
                'messages': 0,
                'nonempty_messages': 0,
                'frame_ids': [],
            },
        }
    }

    passed, failures = evaluate_report(
        report,
        min_messages={'/adascore/shadow/people': 1},
        min_nonempty={'/adascore/shadow/people': 1},
        required_frames={'/adascore/shadow/people': 'adascore_bag_map'},
    )

    assert not passed
    assert len(failures) == 3


def test_adascore_preflight_fails_wrong_people_topic_type():
    report = evaluate_topics(
        ['/people', '/tf', '/map'],
        required_topics=['/people', '/tf', '/map'],
        topic_types={'/people': ['std_msgs/msg/String']},
    )

    assert report['summary']['required_topics_available']
    assert not report['summary']['expected_topic_types_match']
    assert report['wrong_type_topics'] == ['/people']
    assert preflight_exit_code(report, fail_on_missing=True) == 2


def test_parse_expected_type_accepts_topic_equals_type_list():
    topic, types = parse_expected_type('people=people_msgs/msg/People,std_msgs/msg/String')

    assert topic == '/people'
    assert types == ['people_msgs/msg/People', 'std_msgs/msg/String']


def test_shadow_controller_config_uses_social_force_and_shadow_people():
    config = yaml.safe_load((PACKAGE_ROOT / 'config' / 'adascore_shadow_controller.yaml').read_text())
    params = config['controller_server']['ros__parameters']
    follow_path = params['FollowPath']
    sensor_interface = follow_path['sensor_interface']

    assert params['controller_plugins'] == ['FollowPath']
    assert follow_path['plugin'] == 'social_force_window_planner::SFWPlannerNode'
    assert follow_path['controller_frame'] == 'odom'
    assert follow_path['robot_base_frame'] == 'base_link'
    assert sensor_interface['people_topic'] == '/adascore/shadow/people'
    assert sensor_interface['laser_topic'] == '/scan'
    assert sensor_interface['odom_topic'] == '/rko_lio/odometry'
    assert params['odom_topic'] == '/rko_lio/odometry'
    assert params['enable_stamped_cmd_vel'] is False


def test_shadow_controller_launch_remaps_cmd_vel_off_live_topic():
    launch_text = (PACKAGE_ROOT / 'launch' / 'adascore_shadow_controller.launch.py').read_text()

    assert "package='nav2_controller'" in launch_text
    assert "executable='controller_server'" in launch_text
    assert "'/adascore/shadow/cmd_vel'" in launch_text
    assert "('/cmd_vel', '/adascore/shadow/cmd_vel')" in launch_text
