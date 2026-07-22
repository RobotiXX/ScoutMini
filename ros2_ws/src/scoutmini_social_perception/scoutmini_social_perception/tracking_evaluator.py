"""Evaluate recorded tracker output and render human-review event reels."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import defaultdict
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import subprocess
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray

from .bag_io import iter_serialized_messages, read_bag_metadata, topic_counts
from .corpus_manifest import IMAGE_TOPIC, REFERENCE_TRACKS_TOPIC


TRACKS_TOPIC = '/people/tracks_2d'
DIAGNOSTICS_TOPIC = '/people/detector_diagnostics'


@dataclass(frozen=True)
class BoxObservation:
    """One tracker observation in source-image coordinates."""

    track_id: str
    box: Tuple[float, float, float, float]
    confidence: float


@dataclass(frozen=True)
class TrackFrame:
    """All observations published for one source frame."""

    stamp_ns: int
    observations: Tuple[BoxObservation, ...]


class TimedFrames:
    """Nearest-time lookup for an ordered sequence of track frames."""

    def __init__(self, frames: Sequence[TrackFrame]) -> None:
        self.frames = sorted(frames, key=lambda frame: frame.stamp_ns)
        self.stamps = [frame.stamp_ns for frame in self.frames]

    def nearest(
        self,
        stamp_ns: int,
        tolerance_sec: float,
    ) -> Optional[TrackFrame]:
        """Return the nearest frame within the given tolerance."""
        if not self.stamps:
            return None
        index = bisect_left(self.stamps, stamp_ns)
        candidates = []
        if index < len(self.stamps):
            candidates.append(index)
        if index:
            candidates.append(index - 1)
        nearest = min(
            candidates,
            key=lambda candidate: abs(self.stamps[candidate] - stamp_ns),
        )
        if abs(self.stamps[nearest] - stamp_ns) > tolerance_sec * 1e9:
            return None
        return self.frames[nearest]


def _stamp_ns(message, fallback_ns: int) -> int:
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return fallback_ns
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value or fallback_ns


def _observation(detection) -> BoxObservation:
    center = detection.bbox.center.position
    half_width = 0.5 * detection.bbox.size_x
    half_height = 0.5 * detection.bbox.size_y
    confidence = (
        float(detection.results[0].hypothesis.score)
        if detection.results else 0.0
    )
    return BoxObservation(
        track_id=str(detection.id),
        box=(
            float(center.x - half_width),
            float(center.y - half_height),
            float(center.x + half_width),
            float(center.y + half_height),
        ),
        confidence=confidence,
    )


def load_track_frames(bag: Path, topic: str) -> List[TrackFrame]:
    """Load typed Detection2DArray frames directly from a rosbag."""
    frames = []
    for record in iter_serialized_messages(bag, [topic]):
        message = deserialize_message(record.data, Detection2DArray)
        frames.append(TrackFrame(
            stamp_ns=_stamp_ns(message, record.received_ns),
            observations=tuple(
                _observation(detection) for detection in message.detections
            ),
        ))
    return sorted(frames, key=lambda frame: frame.stamp_ns)


def intersection_over_union(first, second) -> float:
    """Return axis-aligned intersection over union."""
    ax1, ay1, ax2, ay2 = first
    bx1, by1, bx2, by2 = second
    width = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    height = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = width * height
    union = (
        (ax2 - ax1) * (ay2 - ay1) +
        (bx2 - bx1) * (by2 - by1) - intersection
    )
    return intersection / union if union > 0.0 else 0.0


def greedy_matches(
    first: Iterable[BoxObservation],
    second: Iterable[BoxObservation],
    minimum_iou: float = 0.3,
) -> List[Tuple[BoxObservation, BoxObservation, float]]:
    """Return one-to-one IoU matches in descending quality order."""
    first = list(first)
    second = list(second)
    candidates = []
    for first_index, first_item in enumerate(first):
        for second_index, second_item in enumerate(second):
            iou = intersection_over_union(first_item.box, second_item.box)
            candidates.append((iou, first_index, second_index))
    matches = []
    used_first = set()
    used_second = set()
    for iou, first_index, second_index in sorted(candidates, reverse=True):
        if iou < minimum_iou:
            break
        if first_index in used_first or second_index in used_second:
            continue
        used_first.add(first_index)
        used_second.add(second_index)
        matches.append((first[first_index], second[second_index], iou))
    return matches


def _center_distance(first, second, image_size) -> float:
    first_x = 0.5 * (first.box[0] + first.box[2]) / image_size[0]
    first_y = 0.5 * (first.box[1] + first.box[3]) / image_size[1]
    second_x = 0.5 * (second.box[0] + second.box[2]) / image_size[0]
    second_y = 0.5 * (second.box[1] + second.box[3]) / image_size[1]
    return math.hypot(first_x - second_x, first_y - second_y)


def _event(
    event_type: str,
    stamp_ns: int,
    before_id: str,
    after_id: str,
    score: float,
    reference_id: str = '',
) -> dict:
    return {
        'event_type': event_type,
        'timestamp_ns': stamp_ns,
        'timestamp_sec': round(stamp_ns / 1e9, 6),
        'before_id': before_id,
        'after_id': after_id,
        'reference_id': reference_id,
        'score': round(float(score), 6),
    }


def evaluate_tracks(
    current: Sequence[TrackFrame],
    reference: Sequence[TrackFrame],
    image_size=(2880, 1440),
) -> Tuple[dict, List[dict]]:
    """Compute intrinsic metrics and suspected identity-failure events."""
    histories: Dict[str, List[Tuple[int, BoxObservation]]] = defaultdict(list)
    events = []
    frame_deltas = [
        (second.stamp_ns - first.stamp_ns) / 1e9
        for first, second in zip(current, current[1:])
        if second.stamp_ns > first.stamp_ns
    ]
    frame_period = statistics.median(frame_deltas) if frame_deltas else 0.125
    gap_threshold = max(0.5, frame_period * 2.5)

    for frame_index, frame in enumerate(current):
        active_ids = {item.track_id for item in frame.observations}
        for item in frame.observations:
            history = histories[item.track_id]
            if history:
                previous_stamp, previous = history[-1]
                elapsed = (frame.stamp_ns - previous_stamp) / 1e9
                distance = _center_distance(previous, item, image_size)
                if elapsed <= 0.5 and distance >= 0.20:
                    events.append(_event(
                        'same_id_spatial_jump',
                        frame.stamp_ns,
                        item.track_id,
                        item.track_id,
                        distance,
                    ))
            elif frame_index:
                candidates = []
                for old_id, old_history in histories.items():
                    if old_id in active_ids or not old_history:
                        continue
                    old_stamp, old_item = old_history[-1]
                    elapsed = (frame.stamp_ns - old_stamp) / 1e9
                    if not 0.0 < elapsed <= 1.5:
                        continue
                    distance = _center_distance(old_item, item, image_size)
                    if distance <= 0.12:
                        candidates.append((distance, elapsed, old_id))
                if candidates:
                    distance, _, old_id = min(candidates)
                    events.append(_event(
                        'suspected_id_fragmentation',
                        frame.stamp_ns,
                        old_id,
                        item.track_id,
                        1.0 - distance,
                    ))
            history.append((frame.stamp_ns, item))

    gap_count = 0
    lifetimes = []
    observations_per_id = {}
    for track_id, history in histories.items():
        observations_per_id[track_id] = len(history)
        lifetimes.append((history[-1][0] - history[0][0]) / 1e9)
        gap_count += sum(
            (second[0] - first[0]) / 1e9 > gap_threshold
            for first, second in zip(history, history[1:])
        )

    reference_lookup = TimedFrames(reference)
    matched_ious = []
    matched_current = 0
    matched_reference = 0
    total_reference = 0
    reference_associations = defaultdict(set)
    last_reference_match = {}
    for frame in current:
        reference_frame = reference_lookup.nearest(frame.stamp_ns, 0.075)
        if reference_frame is None:
            continue
        total_reference += len(reference_frame.observations)
        matches = greedy_matches(
            frame.observations,
            reference_frame.observations,
        )
        matched_current += len(matches)
        matched_reference += len(matches)
        for current_item, reference_item, iou in matches:
            matched_ious.append(iou)
            reference_associations[reference_item.track_id].add(
                current_item.track_id
            )
            previous = last_reference_match.get(reference_item.track_id)
            if previous is not None:
                previous_stamp, previous_current_id = previous
                elapsed = (frame.stamp_ns - previous_stamp) / 1e9
                changed_id = previous_current_id != current_item.track_id
                if elapsed <= 0.75 and changed_id:
                    events.append(_event(
                        'baseline_association_change',
                        frame.stamp_ns,
                        previous_current_id,
                        current_item.track_id,
                        iou,
                        reference_item.track_id,
                    ))
            last_reference_match[reference_item.track_id] = (
                frame.stamp_ns,
                current_item.track_id,
            )

    total_current = sum(len(frame.observations) for frame in current)
    counts = list(observations_per_id.values())
    metrics = {
        'frames': len(current),
        'frames_with_people': sum(
            bool(frame.observations) for frame in current
        ),
        'detections': total_current,
        'unique_public_ids': len(histories),
        'singleton_public_ids': sum(count == 1 for count in counts),
        'short_public_ids_le_3': sum(count <= 3 for count in counts),
        'median_observations_per_id': (
            statistics.median(counts) if counts else None
        ),
        'median_track_lifetime_sec': (
            round(statistics.median(lifetimes), 6) if lifetimes else None
        ),
        'within_id_temporal_gaps': gap_count,
        'suspected_fragmentations': sum(
            event['event_type'] == 'suspected_id_fragmentation'
            for event in events
        ),
        'same_id_spatial_jumps': sum(
            event['event_type'] == 'same_id_spatial_jump'
            for event in events
        ),
        'reference_available': bool(reference),
        'reference_detections_at_matched_frames': total_reference,
        'baseline_association_changes': sum(
            event['event_type'] == 'baseline_association_change'
            for event in events
        ),
        'baseline_ids_split_across_multiple_current_ids': sum(
            len(values) > 1 for values in reference_associations.values()
        ),
        'matched_box_pairs': len(matched_ious),
        'mean_matched_iou': (
            round(statistics.fmean(matched_ious), 6)
            if matched_ious else None
        ),
        'current_detection_match_fraction': (
            round(matched_current / total_current, 6)
            if total_current and reference else None
        ),
        'baseline_detection_match_fraction': (
            round(matched_reference / total_reference, 6)
            if total_reference else None
        ),
        'observations_per_id': observations_per_id,
    }
    events.sort(key=lambda item: (item['timestamp_ns'], item['event_type']))
    return metrics, events


class FfmpegWriter:
    """Write BGR frames to a broadly compatible H.264 file."""

    def __init__(self, path: Path, size, fps: float) -> None:
        command = [
            'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', f'{size[0]}x{size[1]}',
            '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
            '-preset', 'fast', '-crf', '21', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(path),
        ]
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE)

    def write(self, frame) -> None:
        self._process.stdin.write(np.ascontiguousarray(frame).tobytes())

    def close(self) -> None:
        self._process.stdin.close()
        return_code = self._process.wait()
        if return_code:
            raise RuntimeError(f'ffmpeg failed with exit code {return_code}')


def _draw_observations(image, observations, color, prefix='') -> None:
    for observation in observations:
        x1, y1, x2, y2 = [int(round(value)) for value in observation.box]
        x1 = max(0, min(image.shape[1] - 1, x1))
        y1 = max(0, min(image.shape[0] - 1, y1))
        x2 = max(0, min(image.shape[1] - 1, x2))
        y2 = max(0, min(image.shape[0] - 1, y2))
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 3)
        cv2.putText(
            image,
            f'{prefix}{observation.track_id}',
            (x1 + 3, max(30, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )


def _selected_events(events, frames, maximum: int) -> List[dict]:
    priority = {
        'baseline_association_change': 0,
        'suspected_id_fragmentation': 1,
        'same_id_spatial_jump': 2,
    }
    ranked = sorted(
        events,
        key=lambda event: (
            priority.get(event['event_type'], 9),
            -event['score'],
            event['timestamp_ns'],
        ),
    )
    selected = []
    for event in ranked:
        if any(
            abs(event['timestamp_ns'] - item['timestamp_ns']) < 1_500_000_000
            for item in selected
        ):
            continue
        selected.append(event)
        if len(selected) >= maximum:
            return selected
    if frames:
        controls = [0.25, 0.5, 0.75]
        for fraction in controls:
            frame = frames[min(len(frames) - 1, int(len(frames) * fraction))]
            if any(
                abs(frame.stamp_ns - item['timestamp_ns']) < 1_500_000_000
                for item in selected
            ):
                continue
            selected.append(_event(
                'control_window', frame.stamp_ns, '', '', 0.0
            ))
            if len(selected) >= maximum:
                break
    return sorted(selected, key=lambda item: item['timestamp_ns'])


def render_review_reel(
    source_bag: Path,
    current: TimedFrames,
    reference: TimedFrames,
    selected: Sequence[dict],
    output: Path,
    fps: float,
) -> int:
    """Render event windows with current and optional baseline overlays."""
    if not selected:
        return 0
    writer = FfmpegWriter(output, (1920, 960), fps)
    written = 0
    last_written = {index: None for index in range(len(selected))}
    try:
        for record in iter_serialized_messages(source_bag, [IMAGE_TOPIC]):
            message = deserialize_message(record.data, CompressedImage)
            stamp_ns = _stamp_ns(message, record.received_ns)
            event_index = next((
                index for index, event in enumerate(selected)
                if abs(stamp_ns - event['timestamp_ns']) <= 1_250_000_000
            ), None)
            if event_index is None:
                continue
            previous_stamp = last_written[event_index]
            if previous_stamp is not None:
                if stamp_ns - previous_stamp < int(1e9 / fps):
                    continue
            last_written[event_index] = stamp_ns
            image = cv2.imdecode(
                np.frombuffer(message.data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if image is None:
                continue
            current_frame = current.nearest(stamp_ns, 0.075)
            reference_frame = reference.nearest(stamp_ns, 0.075)
            if current_frame is not None:
                _draw_observations(
                    image,
                    current_frame.observations,
                    (40, 220, 40),
                )
            if reference_frame is not None:
                _draw_observations(
                    image,
                    reference_frame.observations,
                    (230, 205, 20),
                    'B:',
                )
            image = cv2.resize(image, (1920, 960))
            event = selected[event_index]
            relative = (stamp_ns - event['timestamp_ns']) / 1e9
            cv2.rectangle(image, (0, 0), (1920, 72), (20, 22, 25), -1)
            title = (
                f"REVIEW {event_index + 1}/{len(selected)} | "
                f"{event['event_type']} | dt={relative:+.2f}s"
            )
            cv2.putText(
                image, title, (20, 32), cv2.FONT_HERSHEY_SIMPLEX,
                0.72, (245, 245, 245), 2, cv2.LINE_AA,
            )
            identity = (
                f"current {event['before_id']} -> {event['after_id']} | "
                f"baseline {event['reference_id'] or 'n/a'} | REVIEW REQUIRED"
            )
            cv2.putText(
                image, identity, (20, 61), cv2.FONT_HERSHEY_SIMPLEX,
                0.58, (80, 210, 255), 2, cv2.LINE_AA,
            )
            writer.write(image)
            written += 1
    finally:
        writer.close()
    return written


def evaluate(args) -> dict:
    """Evaluate one current-output bag and write review artifacts."""
    source_bag = Path(args.source_bag).resolve()
    analysis_bag = Path(args.analysis_bag).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    scene = args.scene or source_bag.name
    current_frames = load_track_frames(analysis_bag, TRACKS_TOPIC)
    if not current_frames:
        raise RuntimeError(f'No {TRACKS_TOPIC} messages in {analysis_bag}')
    source_topics = topic_counts(read_bag_metadata(source_bag))
    reference_frames = []
    if source_topics.get(REFERENCE_TRACKS_TOPIC, {}).get('count', 0):
        reference_frames = load_track_frames(
            source_bag,
            REFERENCE_TRACKS_TOPIC,
        )
    metrics, events = evaluate_tracks(current_frames, reference_frames)
    selected = _selected_events(events, current_frames, args.max_events)
    reel_path = output_dir / f'{scene}_tracking_review.mp4'
    rendered_frames = render_review_reel(
        source_bag,
        TimedFrames(current_frames),
        TimedFrames(reference_frames),
        selected,
        reel_path,
        args.fps,
    )
    metrics.update({
        'scene': scene,
        'source_bag': str(source_bag),
        'analysis_bag': str(analysis_bag),
        'candidate_events': len(events),
        'selected_review_events': len(selected),
        'review_reel_frames': rendered_frames,
        'review_reel': str(reel_path) if rendered_frames else None,
        'claims': [
            'Intrinsic continuity metrics are automated diagnostics, not '
            'identity ground truth.',
            'Recorded blue tracks are a descriptive baseline, not human '
            'labels.',
            'Suspected events require a human verdict before being counted '
            'as ID switches.',
        ],
    })
    metrics_path = output_dir / f'{scene}_tracking_metrics.json'
    with metrics_path.open('w', encoding='utf-8') as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
        stream.write('\n')
    event_fields = [
        'event_type', 'timestamp_ns', 'timestamp_sec', 'before_id',
        'after_id', 'reference_id', 'score',
    ]
    with (output_dir / f'{scene}_tracking_events.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=event_fields)
        writer.writeheader()
        writer.writerows(events)
    review_fields = event_fields + ['verdict', 'reviewer', 'notes']
    with (output_dir / f'{scene}_review_labels.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=review_fields)
        writer.writeheader()
        for event in selected:
            writer.writerow({
                **event,
                'verdict': '',
                'reviewer': '',
                'notes': '',
            })
    return metrics


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-bag', required=True)
    parser.add_argument('--analysis-bag', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--scene', default='')
    parser.add_argument('--fps', type=float, default=8.0)
    parser.add_argument('--max-events', type=int, default=12)
    return parser.parse_args(argv)


def main() -> None:
    """CLI entry point."""
    metrics = evaluate(parse_args())
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
