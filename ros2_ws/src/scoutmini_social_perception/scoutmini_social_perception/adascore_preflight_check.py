"""Read-only ROS graph preflight for AdaSCoRe/Nav2 dry integration."""

from __future__ import annotations

import argparse
import json
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


def normalize_topic_name(topic: str) -> str:
    stripped = topic.strip()
    if not stripped:
        return ''
    return stripped if stripped.startswith('/') else f'/{stripped}'


def evaluate_topics(
    topic_names: Iterable[str],
    required_topics: Iterable[str] = DEFAULT_REQUIRED_TOPICS,
    motion_topics: Iterable[str] = DEFAULT_MOTION_TOPICS,
) -> dict[str, object]:
    present = {normalize_topic_name(topic) for topic in topic_names if normalize_topic_name(topic)}
    required = [normalize_topic_name(topic) for topic in required_topics]
    motion = [normalize_topic_name(topic) for topic in motion_topics]
    missing_required = [topic for topic in required if topic not in present]
    present_motion = [topic for topic in motion if topic in present]
    return {
        'required_topics': {topic: topic in present for topic in required},
        'missing_required_topics': missing_required,
        'motion_topics_present': present_motion,
        'summary': {
            'required_topics_available': not missing_required,
            'motion_topics_detected': bool(present_motion),
            'safe_to_start_motion': False,
        },
    }


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
    args = parser.parse_args()

    required_topics = _parse_csv(args.required_topic) or DEFAULT_REQUIRED_TOPICS
    motion_topics = _parse_csv(args.motion_topic) or DEFAULT_MOTION_TOPICS

    rclpy.init()
    node = _GraphProbe()
    try:
        end_time = time.monotonic() + max(0.0, args.settle_sec)
        while time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        topics = [name for name, _types in node.get_topic_names_and_types()]
        report = evaluate_topics(topics, required_topics, motion_topics)
        report['observed_topic_count'] = len(topics)
        print(json.dumps(report, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
