"""Collect pass/fail evidence from ScoutMini perception bag replay topics."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .track_schema import parse_people_frame


DEFAULT_STRING_TOPICS = [
    '/people/detector_metrics',
    '/people/detections_2d',
    '/people/tracks_2d',
    '/people/projected',
    '/adascore/people_debug',
]


def _load_people_msg_type() -> type | None:
    try:
        from people_msgs.msg import People

        return People
    except Exception:
        return None


def _normalize_topic(topic: str) -> str:
    topic = topic.strip()
    if not topic:
        return ''
    return topic if topic.startswith('/') else f'/{topic}'


def _parse_topic_int(value: str) -> tuple[str, int]:
    if '=' not in value:
        raise argparse.ArgumentTypeError('Expected /topic=count.')
    topic, raw_count = value.split('=', 1)
    try:
        count = int(raw_count)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Count must be an integer.') from exc
    topic = _normalize_topic(topic)
    if not topic or count < 0:
        raise argparse.ArgumentTypeError('Expected a non-empty topic and non-negative count.')
    return topic, count


def _parse_topic_frame(value: str) -> tuple[str, str]:
    if '=' not in value:
        raise argparse.ArgumentTypeError('Expected /topic=frame_id.')
    topic, frame = value.split('=', 1)
    topic = _normalize_topic(topic)
    frame = frame.strip()
    if not topic or not frame:
        raise argparse.ArgumentTypeError('Expected a non-empty topic and frame_id.')
    return topic, frame


def evaluate_report(
    report: dict[str, Any],
    min_messages: dict[str, int] | None = None,
    min_nonempty: dict[str, int] | None = None,
    required_frames: dict[str, str] | None = None,
) -> tuple[bool, list[str]]:
    failures: list[str] = []
    topics = report.get('topics', {})
    min_messages = min_messages or {}
    min_nonempty = min_nonempty or {}
    required_frames = required_frames or {}

    for topic, minimum in min_messages.items():
        actual = int(topics.get(topic, {}).get('messages', 0))
        if actual < minimum:
            failures.append(f'{topic} messages {actual} < required {minimum}')

    for topic, minimum in min_nonempty.items():
        actual = int(topics.get(topic, {}).get('nonempty_messages', 0))
        if actual < minimum:
            failures.append(f'{topic} nonempty_messages {actual} < required {minimum}')

    for topic, frame in required_frames.items():
        observed = set(topics.get(topic, {}).get('frame_ids', []))
        if frame not in observed:
            failures.append(f'{topic} missing required frame {frame!r}; observed {sorted(observed)!r}')

    return not failures, failures


class PerceptionBagValidator(Node):
    def __init__(self, string_topics: list[str], people_topics: list[str]) -> None:
        super().__init__('perception_bag_validate')
        self.topic_stats: dict[str, dict[str, Any]] = {}
        self._metric_elapsed: list[float] = []

        for topic in string_topics:
            normalized = _normalize_topic(topic)
            if not normalized:
                continue
            self.topic_stats[normalized] = self._new_topic_stats('std_msgs/msg/String')
            self.create_subscription(String, normalized, self._string_callback(normalized), 10)

        people_msg_type = _load_people_msg_type()
        for topic in people_topics:
            normalized = _normalize_topic(topic)
            if not normalized:
                continue
            self.topic_stats[normalized] = self._new_topic_stats(
                'people_msgs/msg/People' if people_msg_type is not None else 'missing people_msgs'
            )
            if people_msg_type is not None:
                self.create_subscription(people_msg_type, normalized, self._people_callback(normalized), 10)

        self.people_msgs_available = people_msg_type is not None
        self.get_logger().info(f'Validating string topics: {string_topics}')
        self.get_logger().info(f'Validating people_msgs topics: {people_topics}')

    @staticmethod
    def _new_topic_stats(topic_type: str) -> dict[str, Any]:
        return {
            'type': topic_type,
            'messages': 0,
            'nonempty_messages': 0,
            'total_people': 0,
            'max_people': 0,
            'frame_ids': [],
            'schema_errors': 0,
            'last_people_count': 0,
        }

    def _record_people_count(self, topic: str, frame_id: str, people_count: int) -> None:
        stats = self.topic_stats[topic]
        stats['last_people_count'] = people_count
        stats['total_people'] += people_count
        stats['max_people'] = max(int(stats['max_people']), people_count)
        if people_count > 0:
            stats['nonempty_messages'] += 1
        if frame_id and frame_id not in stats['frame_ids']:
            stats['frame_ids'].append(frame_id)

    def _string_callback(self, topic: str):
        def callback(msg: String) -> None:
            stats = self.topic_stats[topic]
            stats['messages'] += 1
            try:
                if topic.endswith('detector_metrics'):
                    metrics = json.loads(msg.data)
                    if 'elapsed_ms' in metrics:
                        self._metric_elapsed.append(float(metrics['elapsed_ms']))
                    frame_id = str(metrics.get('frame_id', ''))
                    if frame_id and frame_id not in stats['frame_ids']:
                        stats['frame_ids'].append(frame_id)
                    if int(metrics.get('detection_count', 0)) > 0:
                        stats['nonempty_messages'] += 1
                    return

                frame = parse_people_frame(msg.data)
                self._record_people_count(topic, frame.frame_id, len(frame.people))
            except Exception:
                stats['schema_errors'] += 1

        return callback

    def _people_callback(self, topic: str):
        def callback(msg: Any) -> None:
            stats = self.topic_stats[topic]
            stats['messages'] += 1
            frame_id = str(getattr(msg.header, 'frame_id', ''))
            people = list(getattr(msg, 'people', []))
            self._record_people_count(topic, frame_id, len(people))

        return callback

    def report(self, duration_sec: float) -> dict[str, Any]:
        topics = {}
        for topic, stats in sorted(self.topic_stats.items()):
            out = dict(stats)
            messages = max(0, int(out['messages']))
            out['average_people'] = round(float(out['total_people']) / messages, 3) if messages else 0.0
            out['hz'] = round(float(messages) / max(duration_sec, 1e-6), 3)
            out['frame_ids'] = sorted(out['frame_ids'])
            topics[topic] = out

        metrics = {}
        if self._metric_elapsed:
            metrics = {
                'elapsed_ms_min': round(min(self._metric_elapsed), 3),
                'elapsed_ms_mean': round(statistics.mean(self._metric_elapsed), 3),
                'elapsed_ms_max': round(max(self._metric_elapsed), 3),
            }

        return {
            'duration_sec': round(duration_sec, 3),
            'people_msgs_available': self.people_msgs_available,
            'topics': topics,
            'detector_metrics': metrics,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description='Validate ScoutMini perception outputs during bag replay.')
    parser.add_argument('--duration-sec', type=float, default=15.0)
    parser.add_argument('--string-topic', action='append', default=[])
    parser.add_argument('--people-topic', action='append', default=[])
    parser.add_argument('--min-messages', action='append', type=_parse_topic_int, default=[])
    parser.add_argument('--min-nonempty', action='append', type=_parse_topic_int, default=[])
    parser.add_argument('--require-frame', action='append', type=_parse_topic_frame, default=[])
    parser.add_argument('--fail-on-missing', action='store_true')
    args = parser.parse_args()

    string_topics = args.string_topic or DEFAULT_STRING_TOPICS
    people_topics = args.people_topic or []

    rclpy.init()
    node = PerceptionBagValidator(string_topics, people_topics)
    start = time.monotonic()
    try:
        end = start + max(0.1, args.duration_sec)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        duration = time.monotonic() - start
        report = node.report(duration)
        passed, failures = evaluate_report(
            report,
            min_messages=dict(args.min_messages),
            min_nonempty=dict(args.min_nonempty),
            required_frames=dict(args.require_frame),
        )
        report['summary'] = {
            'passed': passed,
            'failures': failures,
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        sys.exit(0 if passed or not args.fail_on_missing else 2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
