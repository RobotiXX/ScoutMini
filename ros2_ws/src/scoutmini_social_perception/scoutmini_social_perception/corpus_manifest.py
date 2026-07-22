"""Inventory local Insta360 rosbag2 recordings for reproducible evaluation."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Dict, Iterable, Optional

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .bag_io import (
    edge_serialized_messages,
    read_bag_metadata,
    storage_integrity,
    topic_counts,
)


IMAGE_TOPIC = '/insta360_x4/image_raw/compressed'
CAMERA_INFO_TOPIC = '/insta360_x4/camera_info'
REFERENCE_TRACKS_TOPIC = '/tracks_insta360_x4_image_raw_compressed'
KEYPOINTS_TOPIC = '/keypoints_insta360_x4_image_raw_compressed'
POINTCLOUD_TOPIC = '/velodyne_points'
ODOMETRY_TOPIC = '/odom'
TF_TOPIC = '/tf'
TF_STATIC_TOPIC = '/tf_static'


def _header_stamp_ns(message) -> Optional[int]:
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value or None


def _timestamp_quality(
    bag: Path,
    topics: Dict[str, Dict[str, object]],
) -> dict:
    wanted = [
        topic for topic in (IMAGE_TOPIC, ODOMETRY_TOPIC, POINTCLOUD_TOPIC)
        if topics.get(topic, {}).get('count', 0)
    ]
    samples = {topic: [] for topic in wanted}
    message_types = {
        topic: get_message(str(topics[topic]['type'])) for topic in wanted
    }
    for topic, records in edge_serialized_messages(bag, wanted).items():
        for record in records:
            message = deserialize_message(record.data, message_types[topic])
            samples[topic].append((
                record.received_ns,
                _header_stamp_ns(message),
            ))

    result = {}
    for topic, values in samples.items():
        headers = [header for _, header in values if header is not None]
        receive_span = values[-1][0] - values[0][0] if len(values) > 1 else 0
        header_span = headers[-1] - headers[0] if len(headers) > 1 else 0
        first_delta = (
            abs(headers[0] - values[0][0]) / 1e9 if headers else None
        )
        if not headers:
            classification = 'missing_header_stamp'
        elif first_delta <= 5.0:
            classification = 'aligned'
        elif receive_span and abs(header_span - receive_span) <= max(
            1_000_000_000,
            int(receive_span * 0.05),
        ):
            classification = 'different_epoch_same_rate'
        else:
            classification = 'inconsistent_clock'
        result[topic] = {
            'classification': classification,
            'first_header_receive_delta_sec': (
                round(first_delta, 6) if first_delta is not None else None
            ),
            'receive_span_sec': round(receive_span / 1e9, 6),
            'header_span_sec': round(header_span / 1e9, 6),
        }
    return result


def _group_name(path: Path, roots: Iterable[Path]) -> str:
    for root in roots:
        try:
            relative = path.relative_to(root)
        except ValueError:
            continue
        if len(relative.parts) > 1:
            parent = relative.parent.as_posix()
            if relative.parts[0] == 'bags':
                return parent
            scene = re.sub(r'_\d+_\d+$', '', path.name)
            return f'{parent}/{scene}'
    return path.parent.name


def assign_splits(records, seed: str, holdout_fraction: float) -> None:
    """Assign deterministic grouped splits with at least one holdout group."""
    groups = sorted({record['group'] for record in records})
    if not groups:
        return
    ranked = sorted(
        groups,
        key=lambda group: hashlib.sha256(
            f'{seed}:{group}'.encode()
        ).digest(),
    )
    holdout_count = max(1, int(math.ceil(len(groups) * holdout_fraction)))
    holdout = set(ranked[:holdout_count])
    for record in records:
        record['split'] = (
            'holdout' if record['group'] in holdout else 'tune'
        )


def sample_segments(
    duration_sec: float,
    budget_sec: float,
    window_count: int,
) -> list:
    """Select full short bags or evenly spaced windows from long bags."""
    if duration_sec <= 0.0:
        return []
    if budget_sec <= 0.0 or duration_sec <= budget_sec:
        return [{
            'id': 'full',
            'start_offset_sec': 0.0,
            'duration_sec': round(duration_sec, 6),
        }]
    window_count = max(1, window_count)
    window_duration = budget_sec / window_count
    segments = []
    for index in range(window_count):
        fraction = (index + 0.5) / window_count
        center = duration_sec * fraction
        start = max(0.0, min(
            duration_sec - window_duration,
            center - 0.5 * window_duration,
        ))
        segments.append({
            'id': f'window_{index + 1:02d}',
            'start_offset_sec': round(start, 6),
            'duration_sec': round(window_duration, 6),
        })
    return segments


def inspect_bag(
    bag: Path,
    roots: Iterable[Path],
    check_storage: bool,
) -> dict:
    """Return one JSON-ready corpus record."""
    information = read_bag_metadata(bag)
    topics = topic_counts(information)
    duration_ns = int(information.get('duration', {}).get('nanoseconds', 0))
    duration_sec = duration_ns / 1e9
    image_frames = int(topics.get(IMAGE_TOPIC, {}).get('count', 0))
    group = _group_name(bag, roots)
    if check_storage:
        storage_ok, storage_problems = storage_integrity(bag, information)
    else:
        storage_ok, storage_problems = True, []
    timestamp_quality = _timestamp_quality(bag, topics) if image_frames else {}
    topic_flags = {
        'has_insta360_image': image_frames > 0,
        'has_camera_info': (
            topics.get(CAMERA_INFO_TOPIC, {}).get('count', 0) > 0
        ),
        'has_reference_tracks': (
            topics.get(REFERENCE_TRACKS_TOPIC, {}).get('count', 0) > 0
        ),
        'has_keypoints': topics.get(KEYPOINTS_TOPIC, {}).get('count', 0) > 0,
        'has_pointcloud': topics.get(POINTCLOUD_TOPIC, {}).get('count', 0) > 0,
        'has_odometry': topics.get(ODOMETRY_TOPIC, {}).get('count', 0) > 0,
        'has_tf': topics.get(TF_TOPIC, {}).get('count', 0) > 0,
        'has_tf_static': topics.get(TF_STATIC_TOPIC, {}).get('count', 0) > 0,
    }
    topic_flags['navigation_ready'] = all([
        topic_flags['has_insta360_image'],
        topic_flags['has_camera_info'],
        topic_flags['has_pointcloud'],
        topic_flags['has_odometry'],
    ])
    corpus_id = hashlib.sha256(
        str(bag.resolve()).encode()
    ).hexdigest()[:12]
    return {
        'name': bag.name,
        'corpus_id': corpus_id,
        'path': str(bag.resolve()),
        'group': group,
        'split': '',
        'duration_sec': round(duration_sec, 6),
        'message_count': int(information.get('message_count', 0)),
        'insta360_frames': image_frames,
        'insta360_fps': (
            round(image_frames / duration_sec, 3) if duration_sec else 0.0
        ),
        'storage_ok': storage_ok,
        'storage_problems': storage_problems,
        **topic_flags,
        'timestamp_quality': timestamp_quality,
    }


def build_manifest(args) -> dict:
    """Scan roots, inspect bags, and return a deterministic manifest."""
    roots = [Path(value).resolve() for value in args.root]
    metadata_paths = sorted({
        metadata.resolve()
        for root in roots
        for metadata in root.rglob('metadata.yaml')
        if '/rendered/' not in str(metadata)
    })
    records = []
    failures = []
    for metadata_path in metadata_paths:
        bag = metadata_path.parent
        try:
            record = inspect_bag(
                bag,
                roots,
                args.check_storage,
            )
        except Exception as exc:  # noqa: BLE001
            failures.append({'path': str(bag), 'error': str(exc)})
            continue
        if args.only_insta360 and not record['has_insta360_image']:
            continue
        records.append(record)

    assign_splits(records, args.seed, args.holdout_fraction)
    for record in records:
        record['segments'] = sample_segments(
            record['duration_sec'],
            args.sample_seconds_per_bag,
            args.sample_windows,
        )

    return {
        'schema_version': 1,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'roots': [str(root) for root in roots],
        'seed': args.seed,
        'holdout_fraction': args.holdout_fraction,
        'bags': records,
        'failures': failures,
        'summary': {
            'bags': len(records),
            'duration_sec': round(
                sum(item['duration_sec'] for item in records),
                6,
            ),
            'insta360_frames': sum(
                item['insta360_frames'] for item in records
            ),
            'reference_bags': sum(
                item['has_reference_tracks'] for item in records
            ),
            'navigation_ready_bags': sum(
                item['navigation_ready'] for item in records
            ),
            'holdout_bags': sum(
                item['split'] == 'holdout' for item in records
            ),
            'storage_failures': sum(
                not item['storage_ok'] for item in records
            ),
            'inspection_failures': len(failures),
        },
    }


def _write_csv(path: Path, records) -> None:
    fields = [
        'name', 'path', 'group', 'split', 'duration_sec', 'message_count',
        'insta360_frames', 'insta360_fps', 'storage_ok',
        'has_reference_tracks', 'has_keypoints', 'has_camera_info',
        'has_pointcloud', 'has_odometry', 'has_tf', 'has_tf_static',
        'navigation_ready',
    ]
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=fields,
            extrasaction='ignore',
        )
        writer.writeheader()
        writer.writerows(records)


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', action='append', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--csv', default='')
    parser.add_argument('--seed', default='scoutmini-social-perception-v1')
    parser.add_argument('--holdout-fraction', type=float, default=0.2)
    parser.add_argument('--only-insta360', action='store_true')
    parser.add_argument('--check-storage', action='store_true')
    parser.add_argument('--sample-seconds-per-bag', type=float, default=180.0)
    parser.add_argument('--sample-windows', type=int, default=3)
    args = parser.parse_args(argv)
    if not 0.0 < args.holdout_fraction < 1.0:
        parser.error('--holdout-fraction must be between zero and one')
    if args.sample_windows < 1:
        parser.error('--sample-windows must be positive')
    return args


def main() -> None:
    """CLI entry point."""
    args = parse_args()
    manifest = build_manifest(args)
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', encoding='utf-8') as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write('\n')
    if args.csv:
        csv_path = Path(args.csv).resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        _write_csv(csv_path, manifest['bags'])
    print(json.dumps(manifest['summary'], indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
