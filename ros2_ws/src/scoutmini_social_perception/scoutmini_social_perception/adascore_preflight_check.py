"""Read-only ROS graph preflight for AdaSCoRe/Nav2 dry integration."""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Iterable

import rclpy
from rclpy.node import Node


DEFAULT_REQUIRED_TOPICS = [
    '/people',
    '/tf',
    '/rko_lio/odometry',
    '/scan',
    '/map',
]

DEFAULT_MOTION_TOPICS = [
    '/cmd_vel',
]

DEFAULT_EXPECTED_TOPIC_TYPES = {
    '/people': ['people_msgs/msg/People'],
}


def normalize_topic_name(topic: str) -> str:
    stripped = topic.strip()
    if not stripped:
        return ''
    return stripped if stripped.startswith('/') else f'/{stripped}'


def parse_expected_type(value: str) -> tuple[str, list[str]]:
    if '=' not in value:
        raise argparse.ArgumentTypeError('Expected topic type must use /topic=type[,type] format.')
    topic, raw_types = value.split('=', 1)
    normalized_topic = normalize_topic_name(topic)
    expected_types = [part.strip() for part in raw_types.split(',') if part.strip()]
    if not normalized_topic or not expected_types:
        raise argparse.ArgumentTypeError('Expected topic type must include a topic and at least one type.')
    return normalized_topic, expected_types


def evaluate_topics(
    topic_names: Iterable[str],
    required_topics: Iterable[str] = DEFAULT_REQUIRED_TOPICS,
    motion_topics: Iterable[str] = DEFAULT_MOTION_TOPICS,
    topic_types: dict[str, list[str]] | None = None,
    expected_topic_types: dict[str, list[str]] | None = None,
) -> dict[str, object]:
    present = {normalize_topic_name(topic) for topic in topic_names if normalize_topic_name(topic)}
    required = [normalize_topic_name(topic) for topic in required_topics]
    motion = [normalize_topic_name(topic) for topic in motion_topics]
    normalized_types = {
        normalize_topic_name(topic): types
        for topic, types in (topic_types or {}).items()
        if normalize_topic_name(topic)
    }
    expected_types = {
        normalize_topic_name(topic): types
        for topic, types in (expected_topic_types or DEFAULT_EXPECTED_TOPIC_TYPES).items()
        if normalize_topic_name(topic)
    }
    missing_required = [topic for topic in required if topic not in present]
    present_motion = [topic for topic in motion if topic in present]
    topic_type_checks = {}
    wrong_type_topics = []
    for topic, expected in expected_types.items():
        actual = normalized_types.get(topic, [])
        matches = any(topic_type in actual for topic_type in expected)
        topic_type_checks[topic] = {
            'actual_types': actual,
            'expected_types': expected,
            'matches': matches,
        }
        if topic in present and not matches:
            wrong_type_topics.append(topic)
    return {
        'required_topics': {topic: topic in present for topic in required},
        'missing_required_topics': missing_required,
        'motion_topics_present': present_motion,
        'topic_type_checks': topic_type_checks,
        'wrong_type_topics': wrong_type_topics,
        'summary': {
            'required_topics_available': not missing_required,
            'motion_topics_detected': bool(present_motion),
            'expected_topic_types_match': not wrong_type_topics,
            'safe_to_start_motion': False,
        },
    }


def preflight_exit_code(report: dict[str, object], fail_on_missing: bool) -> int:
    if not fail_on_missing:
        return 0
    summary = report.get('summary', {})
    if (
        isinstance(summary, dict)
        and summary.get('required_topics_available') is True
        and summary.get('expected_topic_types_match') is True
    ):
        return 0
    return 2


class _GraphProbe(Node):
    def __init__(self) -> None:
        super().__init__('adascore_preflight_check')


def _parse_csv(values: list[str]) -> list[str]:
    out: list[str] = []
    for value in values:
        out.extend(part for part in value.split(',') if part.strip())
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description='Read-only AdaSCoRe ROS graph preflight check.')
    parser.add_argument(
        '--required-topic',
        action='append',
        default=[],
        help='Required topic. Can be repeated or comma-separated.',
    )
    parser.add_argument(
        '--motion-topic',
        action='append',
        default=[],
        help='Motion command topic to flag if present. Can be repeated or comma-separated.',
    )
    parser.add_argument(
        '--settle-sec',
        type=float,
        default=1.0,
        help='Seconds to wait for ROS graph discovery.',
    )
    parser.add_argument(
        '--fail-on-missing',
        action='store_true',
        help='Exit with code 2 when any required topic is missing or has the wrong expected type.',
    )
    parser.add_argument(
        '--expected-type',
        action='append',
        type=parse_expected_type,
        default=[],
        help='Expected topic type in /topic=type[,type] format. Can be repeated.',
    )
    args = parser.parse_args()

    required_topics = _parse_csv(args.required_topic) or DEFAULT_REQUIRED_TOPICS
    motion_topics = _parse_csv(args.motion_topic) or DEFAULT_MOTION_TOPICS
    expected_topic_types = dict(args.expected_type) if args.expected_type else DEFAULT_EXPECTED_TOPIC_TYPES

    rclpy.init()
    node = _GraphProbe()
    try:
        end_time = time.monotonic() + max(0.0, args.settle_sec)
        while time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        topic_names_and_types = node.get_topic_names_and_types()
        topics = [name for name, _types in topic_names_and_types]
        topic_types = {name: types for name, types in topic_names_and_types}
        report = evaluate_topics(
            topics,
            required_topics,
            motion_topics,
            topic_types=topic_types,
            expected_topic_types=expected_topic_types,
        )
        report['observed_topic_count'] = len(topics)
        print(json.dumps(report, indent=2, sort_keys=True))
        sys.exit(preflight_exit_code(report, args.fail_on_missing))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
