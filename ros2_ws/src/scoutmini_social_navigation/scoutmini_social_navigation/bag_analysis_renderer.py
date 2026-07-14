"""Render synchronized GMU perception and AdaSCoRe shadow analysis videos."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


SOURCE_IMAGE = '/insta360_x4/image_raw/compressed'
REFERENCE_TRACKS = '/tracks_insta360_x4_image_raw_compressed'
ODOMETRY = '/odom'
TRACKS = '/people/tracks_2d'
PEOPLE = '/adascore/shadow/people'
TRAJECTORIES = '/adascore_shadow/robot_local_trajectories'
LOCAL_PATH = '/adascore_shadow/robot_local_plan'
SHADOW_CMD = '/adascore/shadow/cmd_vel'
OBSTACLES = '/sfm/markers/obstacle_points'
DETECTOR_DIAGNOSTICS = '/people/detector_diagnostics'


def header_stamp_ns(msg) -> Optional[int]:
    """Return a usable top-level or MarkerArray element header stamp."""
    header = getattr(msg, 'header', None)
    if header is None:
        markers = getattr(msg, 'markers', [])
        header = markers[0].header if markers else None
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value or None


def stamp_ns(msg, fallback: int) -> int:
    """Return a message header timestamp when one is available."""
    return header_stamp_ns(msg) or fallback


def align_receive_time(received_ns: int, anchors: Sequence[Tuple[int, int]]) -> int:
    """Map recorder wall time onto source time using neighboring track stamps."""
    if not anchors:
        return received_ns
    received = [item[0] for item in anchors]
    index = bisect_left(received, received_ns)
    if index == 0:
        anchor_received, anchor_source = anchors[0]
        return received_ns + anchor_source - anchor_received
    if index == len(anchors):
        anchor_received, anchor_source = anchors[-1]
        return received_ns + anchor_source - anchor_received
    before_received, before_source = anchors[index - 1]
    after_received, after_source = anchors[index]
    span = after_received - before_received
    if span <= 0:
        return before_source
    fraction = (received_ns - before_received) / span
    return int(round(before_source + fraction * (after_source - before_source)))


class TimedMessages:
    """Nearest-timestamp lookup for a small ROS message sequence."""

    def __init__(self, records: Sequence[Tuple[int, object]]) -> None:
        ordered = sorted(records, key=lambda item: item[0])
        self._stamps = [item[0] for item in ordered]
        self._messages = [item[1] for item in ordered]

    def nearest(self, target: int, tolerance_sec: float = 0.30):
        """Return the nearest message within the requested tolerance."""
        if not self._stamps:
            return None
        index = bisect_left(self._stamps, target)
        candidates = []
        if index < len(self._stamps):
            candidates.append(index)
        if index:
            candidates.append(index - 1)
        nearest = min(candidates, key=lambda value: abs(self._stamps[value] - target))
        if abs(self._stamps[nearest] - target) > tolerance_sec * 1e9:
            return None
        return self._messages[nearest]

    def __len__(self) -> int:
        return len(self._messages)


def read_topics(bag: Path, wanted: Iterable[str]) -> Dict[str, TimedMessages]:
    """Deserialize selected topics from a rosbag2 directory."""
    wanted = set(wanted)
    records: Dict[str, List[Tuple[int, object, bool]]] = {
        topic: [] for topic in wanted
    }
    clock_anchors = []
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    missing = wanted.difference(types)
    if missing:
        print(f'Warning: topics absent from {bag}: {sorted(missing)}')
    message_types = {
        topic: get_message(types[topic]) for topic in wanted.intersection(types)
    }
    while reader.has_next():
        topic, data, received_ns = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(data, message_types[topic])
        header_ns = None if topic == ODOMETRY else header_stamp_ns(msg)
        records[topic].append((header_ns or received_ns, msg, header_ns is not None))
        if topic == TRACKS and header_ns is not None:
            clock_anchors.append((received_ns, header_ns))
    clock_anchors.sort()
    return {
        topic: TimedMessages([
            (
                value if has_header else align_receive_time(value, clock_anchors),
                msg,
            )
            for value, msg, has_header in values
        ])
        for topic, values in records.items()
    }


class FfmpegWriter:
    """Stream BGR frames into a broadly playable H.264 MP4."""

    def __init__(self, path: Path, size: Tuple[int, int], fps: float) -> None:
        self.path = path
        self.size = size
        command = [
            'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', f'{size[0]}x{size[1]}',
            '-r', f'{fps:.6f}', '-i', '-', '-an', '-c:v', 'libx264',
            '-preset', 'medium', '-crf', '20', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(path),
        ]
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE)

    def write(self, frame) -> None:
        """Write exactly one frame."""
        if frame.shape[1::-1] != self.size:
            raise ValueError(f'Unexpected frame size: {frame.shape[1::-1]}')
        self._process.stdin.write(np.ascontiguousarray(frame).tobytes())

    def close(self) -> None:
        """Finish encoding and reject an incomplete output."""
        self._process.stdin.close()
        result = self._process.wait()
        if result:
            raise RuntimeError(f'ffmpeg failed with exit code {result}')


def detection_box(detection) -> Tuple[float, float, float, float]:
    """Convert a Detection2D bounding box to xyxy coordinates."""
    center = detection.bbox.center.position
    half_width = 0.5 * detection.bbox.size_x
    half_height = 0.5 * detection.bbox.size_y
    return (
        center.x - half_width,
        center.y - half_height,
        center.x + half_width,
        center.y + half_height,
    )


def intersection_over_union(first, second) -> float:
    """Return axis-aligned intersection over union."""
    ax1, ay1, ax2, ay2 = first
    bx1, by1, bx2, by2 = second
    width = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    height = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = width * height
    union = ((ax2 - ax1) * (ay2 - ay1) +
             (bx2 - bx1) * (by2 - by1) - intersection)
    return intersection / union if union > 0.0 else 0.0


def greedy_match_iou(ours, reference) -> List[float]:
    """Greedily match boxes for a descriptive, non-ground-truth comparison."""
    candidates = []
    for ours_index, ours_detection in enumerate(ours):
        for ref_index, ref_detection in enumerate(reference):
            iou = intersection_over_union(
                detection_box(ours_detection),
                detection_box(ref_detection),
            )
            candidates.append((iou, ours_index, ref_index))
    used_ours = set()
    used_reference = set()
    matches = []
    for iou, ours_index, ref_index in sorted(candidates, reverse=True):
        if iou < 0.1:
            break
        if ours_index in used_ours or ref_index in used_reference:
            continue
        used_ours.add(ours_index)
        used_reference.add(ref_index)
        matches.append(iou)
    return matches


def yaw_from_odometry(odom) -> float:
    """Extract planar yaw from an Odometry message."""
    q = odom.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _label(image, text: str, origin, color) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.62
    thickness = 2
    (width, height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x = origin[0]
    y = max(height + baseline + 4, origin[1])
    cv2.rectangle(
        image, (x, y - height - baseline - 4), (x + width + 8, y + 2),
        color, -1,
    )
    cv2.putText(
        image, text, (x + 4, y - baseline - 1), font, scale,
        (0, 0, 0), thickness, cv2.LINE_AA,
    )


def draw_tracks(image, detections, color, prefix: str, people=None, odom=None) -> None:
    """Draw traceable Detection2D boxes and labels."""
    height, width = image.shape[:2]
    fused_people = {
        person.name: person for person in getattr(people, 'people', [])
    }
    for detection in detections:
        x1, y1, x2, y2 = detection_box(detection)
        x1 = max(0, min(width - 1, int(round(x1))))
        y1 = max(0, min(height - 1, int(round(y1))))
        x2 = max(0, min(width - 1, int(round(x2))))
        y2 = max(0, min(height - 1, int(round(y2))))
        label = f'{prefix}{detection.id}'
        if detection.results:
            label += f' {detection.results[0].hypothesis.score:.2f}'
        person = fused_people.get(detection.id)
        if person is not None and odom is not None:
            robot = odom.pose.pose.position
            range_m = math.hypot(
                person.position.x - robot.x,
                person.position.y - robot.y,
            )
            label += f' scan~{range_m:.1f}m'
        elif person is not None:
            label += ' range-fused'
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 3)
        _label(image, label, (x1, y1), color)


def world_to_panel(point, odom, center=(256, 430), scale=42.0):
    """Project an odom-frame point into a robot-relative top-down panel."""
    robot = odom.pose.pose.position
    yaw = yaw_from_odometry(odom)
    dx = point.x - robot.x
    dy = point.y - robot.y
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return int(center[0] - left * scale), int(center[1] - forward * scale)


def draw_topdown(people, odom, trajectories, local_path, obstacles, shadow_cmd):
    """Draw actual social-force inputs and published candidate trajectories."""
    panel = np.full((512, 512, 3), (28, 31, 36), dtype=np.uint8)
    center = (256, 430)
    for meter in range(1, 6):
        cv2.circle(panel, center, int(meter * 42), (58, 62, 68), 1)
    cv2.line(panel, (256, 430), (256, 20), (70, 75, 82), 1)
    cv2.line(panel, (20, 430), (492, 430), (70, 75, 82), 1)
    if odom is not None:
        for marker in getattr(trajectories, 'markers', []):
            points = [world_to_panel(point, odom) for point in marker.points]
            if len(points) >= 2:
                cv2.polylines(panel, [np.asarray(points)], False, (235, 145, 55), 2)
        points = [world_to_panel(pose.pose.position, odom)
                  for pose in getattr(local_path, 'poses', [])]
        if len(points) >= 2:
            cv2.polylines(panel, [np.asarray(points)], False, (0, 215, 255), 4)
        for point in getattr(obstacles, 'points', []):
            pixel = world_to_panel(point, odom)
            if 0 <= pixel[0] < 512 and 0 <= pixel[1] < 512:
                cv2.circle(panel, pixel, 2, (40, 70, 220), -1)
        for person in getattr(people, 'people', []):
            pixel = world_to_panel(person.position, odom)
            if not (0 <= pixel[0] < 512 and 0 <= pixel[1] < 512):
                continue
            cv2.circle(panel, pixel, 9, (40, 220, 40), -1)
            yaw = yaw_from_odometry(odom)
            forward_velocity = (
                math.cos(yaw) * person.velocity.x +
                math.sin(yaw) * person.velocity.y
            )
            left_velocity = (
                -math.sin(yaw) * person.velocity.x +
                math.cos(yaw) * person.velocity.y
            )
            velocity_end = (
                int(pixel[0] - left_velocity * 42),
                int(pixel[1] - forward_velocity * 42),
            )
            cv2.arrowedLine(panel, pixel, velocity_end, (40, 220, 40), 2)
            cv2.putText(
                panel, person.name, (pixel[0] + 10, pixel[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 235, 220), 1,
                cv2.LINE_AA,
            )
    cv2.fillPoly(
        panel, [np.asarray([(246, 442), (266, 442), (256, 414)])],
        (245, 245, 245),
    )
    cv2.putText(
        panel, 'AdaSCoRe shadow view', (16, 28), cv2.FONT_HERSHEY_SIMPLEX,
        0.72, (245, 245, 245), 2, cv2.LINE_AA,
    )
    cv2.putText(
        panel, 'NO MOTOR OUTPUT', (16, 55), cv2.FONT_HERSHEY_SIMPLEX,
        0.62, (70, 190, 255), 2, cv2.LINE_AA,
    )
    cv2.putText(
        panel, 'people green | obstacles red', (16, 80),
        cv2.FONT_HERSHEY_SIMPLEX, 0.43, (220, 220, 220), 1, cv2.LINE_AA,
    )
    cv2.putText(
        panel, 'candidates blue | selected yellow', (16, 101),
        cv2.FONT_HERSHEY_SIMPLEX, 0.43, (220, 220, 220), 1, cv2.LINE_AA,
    )
    if shadow_cmd is not None:
        command = f'cmd shadow: v={shadow_cmd.linear.x:+.2f} w={shadow_cmd.angular.z:+.2f}'
        cv2.putText(
            panel, command, (16, 492), cv2.FONT_HERSHEY_SIMPLEX,
            0.53, (230, 230, 230), 1, cv2.LINE_AA,
        )
    return panel


def diagnostic_value(diagnostics, key: str) -> Optional[str]:
    """Look up one diagnostic key."""
    for status in getattr(diagnostics, 'status', []):
        for value in status.values:
            if value.key == key:
                return value.value
    return None


def draw_status_panel(frame_index, relative_sec, tracks, reference, people,
                      trajectories, shadow_cmd, detector_diagnostics):
    """Draw the textual audit panel below the top-down view."""
    panel = np.full((512, 512, 3), (242, 243, 245), dtype=np.uint8)
    values = [
        ('Clip time', f'{relative_sec:.3f} s'),
        ('Rendered frame', str(frame_index)),
        ('Our detections', str(len(tracks.detections))),
        ('Range-fused people', str(len(getattr(people, 'people', [])))),
        ('Recorded baseline', str(len(reference.detections))),
        ('Candidate trajectories', str(len(getattr(trajectories, 'markers', [])))),
        ('Ada status', 'fresh output' if trajectories else 'no fresh output'),
        ('Range calibration', 'UNVERIFIED'),
    ]
    elapsed = diagnostic_value(detector_diagnostics, 'elapsed_ms')
    if elapsed is not None:
        values.append(('TensorRT inference', f'{elapsed} ms'))
    if shadow_cmd is not None:
        values.extend([
            ('Shadow linear', f'{shadow_cmd.linear.x:+.3f} m/s'),
            ('Shadow angular', f'{shadow_cmd.angular.z:+.3f} rad/s'),
        ])
    cv2.putText(
        panel, 'Frame audit', (18, 38), cv2.FONT_HERSHEY_SIMPLEX,
        0.88, (30, 34, 38), 2, cv2.LINE_AA,
    )
    y = 76
    for label, value in values:
        cv2.putText(panel, label, (18, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.53, (74, 78, 84), 1, cv2.LINE_AA)
        cv2.putText(panel, value, (270, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.53, (15, 18, 22), 2, cv2.LINE_AA)
        y += 36
    cv2.putText(
        panel, 'green: current pipeline', (18, 457), cv2.FONT_HERSHEY_SIMPLEX,
        0.52, (15, 135, 15), 2, cv2.LINE_AA,
    )
    cv2.putText(
        panel, 'cyan: recorded baseline (not ground truth)', (18, 486),
        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (145, 115, 0), 1, cv2.LINE_AA,
    )
    return panel


def annotate_title(image, title: str, color) -> None:
    """Add a stable title band to a comparison pane."""
    cv2.rectangle(image, (0, 0), (image.shape[1], 48), (20, 22, 25), -1)
    cv2.putText(image, title, (18, 34), cv2.FONT_HERSHEY_SIMPLEX,
                0.82, color, 2, cv2.LINE_AA)


def scale_detections(message, scale_x: float, scale_y: float):
    """Copy and scale a Detection2DArray for a resized frame."""
    scaled = deepcopy(message)
    for detection in scaled.detections:
        detection.bbox.center.position.x *= scale_x
        detection.bbox.center.position.y *= scale_y
        detection.bbox.size_x *= scale_x
        detection.bbox.size_y *= scale_y
    return scaled


def track_counts_update(counts: Dict[str, int], detections) -> None:
    """Accumulate per-ID observation counts."""
    for detection in detections:
        counts[detection.id] = counts.get(detection.id, 0) + 1


def render(args) -> Dict[str, object]:
    """Render the complete analysis and return summary metrics."""
    source_bag = Path(args.source_bag).resolve()
    analysis_bag = Path(args.analysis_bag).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    scene = args.scene or source_bag.name
    valid_scene_characters = (
        '-_.abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
    )
    if not scene or any(
        character not in valid_scene_characters for character in scene
    ):
        raise ValueError('scene must contain only letters, numbers, dot, dash, or underscore')
    source = read_topics(source_bag, [REFERENCE_TRACKS, ODOMETRY])
    analysis = read_topics(
        analysis_bag,
        [TRACKS, PEOPLE, TRAJECTORIES, LOCAL_PATH, SHADOW_CMD,
         OBSTACLES, DETECTOR_DIAGNOSTICS],
    )
    if not len(analysis[TRACKS]):
        raise RuntimeError('Analysis bag contains no current-pipeline tracks')

    main_path = output_dir / f'{scene}_social_analysis.mp4'
    comparison_path = output_dir / f'{scene}_tracking_comparison.mp4'
    main_writer = FfmpegWriter(main_path, (2560, 1024), args.fps)
    comparison_writer = FfmpegWriter(comparison_path, (2560, 720), args.fps)
    ours_counts: Dict[str, int] = {}
    reference_counts: Dict[str, int] = {}
    rows = []
    all_ious = []
    frame_index = 0
    encoded_frames = 0
    first_timestamp = None
    previous_main = None
    previous_comparison = None
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source_bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    image_type = get_message(topic_types[SOURCE_IMAGE])
    try:
        while reader.has_next():
            topic, data, received_ns = reader.read_next()
            if topic != SOURCE_IMAGE:
                continue
            image_msg = deserialize_message(data, image_type)
            timestamp = stamp_ns(image_msg, received_ns)
            tracks = analysis[TRACKS].nearest(timestamp, 0.001)
            if tracks is None:
                continue
            if first_timestamp is None:
                first_timestamp = timestamp
            relative_sec = (timestamp - first_timestamp) / 1e9
            image = cv2.imdecode(
                np.frombuffer(image_msg.data, dtype=np.uint8), cv2.IMREAD_COLOR,
            )
            if image is None:
                raise RuntimeError(f'JPEG decode failed at {timestamp}')
            reference = source[REFERENCE_TRACKS].nearest(timestamp, 0.05)
            if reference is None:
                reference = type(tracks)()
            people = analysis[PEOPLE].nearest(timestamp, 0.15)
            trajectories = analysis[TRAJECTORIES].nearest(timestamp, 0.35)
            local_path = analysis[LOCAL_PATH].nearest(timestamp, 0.35)
            shadow_cmd = analysis[SHADOW_CMD].nearest(timestamp, 0.35)
            obstacles = analysis[OBSTACLES].nearest(timestamp, 0.35)
            diagnostics = analysis[DETECTOR_DIAGNOSTICS].nearest(timestamp, 0.05)
            odom = source[ODOMETRY].nearest(timestamp, 0.15)

            ours_view = image.copy()
            draw_tracks(
                ours_view,
                tracks.detections,
                (40, 220, 40),
                '',
                people,
                odom,
            )
            main_image = cv2.resize(ours_view, (2048, 1024))
            sidebar = np.vstack([
                draw_topdown(
                    people, odom, trajectories, local_path, obstacles, shadow_cmd,
                ),
                draw_status_panel(
                    frame_index, relative_sec, tracks, reference, people,
                    trajectories, shadow_cmd, diagnostics,
                ),
            ])
            main_frame = np.hstack([main_image, sidebar])

            ours_compare = cv2.resize(image, (1280, 640))
            reference_compare = ours_compare.copy()
            ours_scaled = scale_detections(
                tracks, 1280.0 / image.shape[1], 640.0 / image.shape[0],
            )
            reference_scaled = scale_detections(
                reference, 1280.0 / image.shape[1], 640.0 / image.shape[0],
            )
            draw_tracks(ours_compare, ours_scaled.detections, (40, 220, 40), '')
            draw_tracks(
                reference_compare, reference_scaled.detections, (230, 205, 20), '',
            )
            annotate_title(ours_compare, 'Current pipeline', (40, 220, 40))
            annotate_title(reference_compare,
                           'Recorded baseline - not ground truth', (230, 205, 20))
            comparison = np.full((720, 2560, 3), (20, 22, 25), dtype=np.uint8)
            comparison[80:720, :1280] = ours_compare
            comparison[80:720, 1280:] = reference_compare
            cv2.putText(
                comparison, f't={relative_sec:.3f}s', (20, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (235, 235, 235), 2,
                cv2.LINE_AA,
            )
            target_frame = int(round(relative_sec * args.fps))
            while previous_main is not None and encoded_frames < target_frame:
                main_writer.write(previous_main)
                comparison_writer.write(previous_comparison)
                encoded_frames += 1
            main_writer.write(main_frame)
            comparison_writer.write(comparison)
            previous_main = main_frame
            previous_comparison = comparison
            encoded_frames += 1

            ious = greedy_match_iou(tracks.detections, reference.detections)
            all_ious.extend(ious)
            track_counts_update(ours_counts, tracks.detections)
            track_counts_update(reference_counts, reference.detections)
            rows.append({
                'timestamp_ns': timestamp,
                'our_detections': len(tracks.detections),
                'recorded_baseline_detections': len(reference.detections),
                'range_fused_people': len(getattr(people, 'people', [])),
                'candidate_trajectories': len(getattr(trajectories, 'markers', [])),
                'shadow_linear_mps': getattr(getattr(shadow_cmd, 'linear', None), 'x', 0.0),
                'shadow_angular_radps': getattr(getattr(shadow_cmd, 'angular', None), 'z', 0.0),
                'mean_matched_iou': float(np.mean(ious)) if ious else 0.0,
                'inference_ms': diagnostic_value(diagnostics, 'elapsed_ms'),
            })
            frame_index += 1
    finally:
        main_writer.close()
        comparison_writer.close()

    metrics = {
        'scene': scene,
        'source_bag': str(source_bag),
        'analysis_bag': str(analysis_bag),
        'rendered_frames': encoded_frames,
        'rendered_duration_sec': encoded_frames / args.fps,
        'processed_track_frames': frame_index,
        'fps': args.fps,
        'our_total_detections': sum(ours_counts.values()),
        'our_unique_ids': len(ours_counts),
        'our_singleton_ids': sum(value == 1 for value in ours_counts.values()),
        'our_observations_per_id': ours_counts,
        'recorded_baseline_total_detections': sum(reference_counts.values()),
        'recorded_baseline_unique_ids': len(reference_counts),
        'recorded_baseline_singleton_ids': sum(
            value == 1 for value in reference_counts.values()
        ),
        'mean_matched_iou': float(np.mean(all_ious)) if all_ious else 0.0,
        'frames_with_range_fusion': sum(
            int(row['range_fused_people'] > 0) for row in rows
        ),
        'frames_with_adascore_trajectories': sum(
            int(row['candidate_trajectories'] > 0) for row in rows
        ),
        'nonzero_shadow_commands': sum(
            int(abs(row['shadow_linear_mps']) > 1e-6 or
                abs(row['shadow_angular_radps']) > 1e-6) for row in rows
        ),
        'notes': [
            'Recorded baseline tracks are not human-annotated ground truth.',
            'Range comes from the synchronized VLP16 point cloud converted to LaserScan.',
            'The GMU bag does not provide camera-to-LiDAR calibration; displayed '
            'scan associations are explicitly marked unverified.',
            'AdaSCoRe used a synthetic 4 m forward path in an isolated shadow namespace.',
            'No output was published to the live robot command topic.',
        ],
        'outputs': [str(main_path), str(comparison_path)],
    }
    if metrics['frames_with_adascore_trajectories'] < frame_index:
        metrics['notes'].append(
            'AdaSCoRe output is intentionally shown as missing when no fresh '
            'trajectory was published; the recorded shadow run stalled under '
            'fused social input and was not interpolated.'
        )
    with (output_dir / f'{scene}_analysis_metrics.json').open(
        'w', encoding='utf-8'
    ) as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
        stream.write('\n')
    with (output_dir / f'{scene}_analysis_frames.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)
    return metrics


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-bag', required=True)
    parser.add_argument('--analysis-bag', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--scene', default='')
    parser.add_argument('--fps', type=float, default=8.0)
    return parser.parse_args(argv)


def main() -> None:
    """CLI entry point."""
    metrics = render(parse_args())
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
