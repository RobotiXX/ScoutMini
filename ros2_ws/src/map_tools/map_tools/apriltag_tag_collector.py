#!/usr/bin/env python3
"""Collect stable AprilTag map anchors and save them to JSON.

The node subscribes to apriltag_ros detections, waits for a keyboard trigger,
buffers the next fixed number of detections, checks whether the observed tag
pose is stable in the map frame, and then writes a tag entry into
map_tools/maps/<map_name>/tags.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
from collections import Counter, deque
from pathlib import Path
from statistics import fmean
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class ApriltagTagCollector(Node):
    """ROS 2 node that captures stable AprilTag poses and stores them in JSON.

    The node listens to the detector output topic published by apriltag_ros and
    uses a keyboard-triggered capture session to stabilize the pose estimate for
    a tag before writing it to the map-specific tags.json file.

    Attributes:
        map_name: Current resolved map name.
        output_file: Absolute path to the JSON file used for persistence.
        capture_depth: Number of detection messages to buffer per capture.
        translation_threshold_m: Maximum allowed pose spread in meters.
    """

    def __init__(self) -> None:
        """Initialize parameters, subscriptions, tf2 access, and worker state.

        Raises:
            RuntimeError: If the node cannot resolve a map name before capture.
        """
        super().__init__('apriltag_tag_collector')

        self.declare_parameter('map_name', '')
        self.declare_parameter('map_name_topic', '/map_name')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('output_file', '')
        self.declare_parameter('capture_depth', 10)
        self.declare_parameter('min_detection_hits', 7)
        self.declare_parameter('translation_threshold_m', 0.05)
        self.declare_parameter('capture_wait_timeout_sec', 30.0)
        self.declare_parameter('name_prefix', 'tag')
        self.declare_parameter('map_name_wait_timeout_sec', 5.0)

        self.raw_map_name = self.get_parameter('map_name').get_parameter_value().string_value.strip()
        self.map_name_topic = self.get_parameter('map_name_topic').get_parameter_value().string_value
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value.strip() or 'map'
        self.raw_output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.capture_depth = max(1, self.get_parameter('capture_depth').get_parameter_value().integer_value)
        self.min_detection_hits = max(1, self.get_parameter('min_detection_hits').get_parameter_value().integer_value)
        self.translation_threshold_m = self.get_parameter('translation_threshold_m').get_parameter_value().double_value
        self.capture_wait_timeout_sec = self.get_parameter('capture_wait_timeout_sec').get_parameter_value().double_value
        self.name_prefix = self.get_parameter('name_prefix').get_parameter_value().string_value.strip() or 'tag'
        self.map_name_wait_timeout_sec = self.get_parameter('map_name_wait_timeout_sec').get_parameter_value().double_value

        self.map_name = self.raw_map_name
        self.output_file = ''
        self._initialized = False
        self._startup_time = self.get_clock().now()
        self._capture_lock = threading.Lock()
        self._capture_ready = threading.Event()
        self._capture_active = False
        self._capture_buffer: Deque[Dict[str, object]] = deque(maxlen=self.capture_depth)
        self._shutdown_event = threading.Event()
        self._stdin_is_tty = bool(getattr(sys.stdin, 'isatty', lambda: False)())
        self._input_thread: Optional[threading.Thread] = None

        self.map_name_subscription = self.create_subscription(
            String, self.map_name_topic, self.map_name_cb, self._map_name_qos()
        )
        self.detections_subscription = self.create_subscription(
            AprilTagDetectionArray, self.detections_topic, self.detections_cb, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.startup_timer = self.create_timer(0.1, self._maybe_initialize)

    def _map_name_qos(self) -> QoSProfile:
        """Create the transient-local QoS profile used for map-name handoff.

        Returns:
            QoSProfile: Reliable, latched profile suitable for /map_name.
        """
        return QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

    def map_name_cb(self, msg: String) -> None:
        """Update the active map name from the latched topic.

        Args:
            msg: String message containing the latest map name.
        """
        if self.raw_map_name:
            return

        candidate = msg.data.strip()
        if not candidate or candidate == self.map_name:
            return

        self.map_name = candidate
        if not self._initialized:
            self.get_logger().info(f'Received map name from {self.map_name_topic}: {self.map_name}')

    def _maybe_initialize(self) -> None:
        """Finish startup once the map name is known or the timeout expires.

        The collector can run without an explicit map name parameter as long as
        the /map_name topic is published before the timeout.
        """
        if self._initialized:
            return

        if not self.map_name:
            elapsed_sec = (self.get_clock().now() - self._startup_time).nanoseconds / 1e9
            if elapsed_sec < self.map_name_wait_timeout_sec:
                return

            self.map_name = 'default_map'
            self.get_logger().warn(
                f'No map name received on {self.map_name_topic}; falling back to {self.map_name}'
            )

        self.output_file = self._resolve_output_file(self.raw_output_file)
        self._initialized = True

        self.get_logger().info('=' * 60)
        self.get_logger().info('AprilTag Tag Collector Started')
        self.get_logger().info(f'Subscribed to detections: {self.detections_topic}')
        self.get_logger().info(f'Using map name: {self.map_name}')
        self.get_logger().info(f'Saving tag anchors to: {self.output_file}')
        self.get_logger().info(f'Capture depth: {self.capture_depth}')
        self.get_logger().info(f'Stable pose threshold: {self.translation_threshold_m:.3f} m')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Press Enter in this terminal to capture the next tag.')
        self.get_logger().info('The node will buffer the next detection window, then ask for a description.')

        if self._stdin_is_tty:
            self._input_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._input_thread.start()
        else:
            self.get_logger().warn('No TTY detected. Keyboard-triggered capture is unavailable in this session.')

        self.startup_timer.cancel()

    def detections_cb(self, msg: AprilTagDetectionArray) -> None:
        """Buffer detections while a capture session is active.

        Args:
            msg: AprilTag detection array from apriltag_ros.
        """
        if not self._capture_active:
            return

        detections = self._extract_detections(msg)
        if not detections:
            return

        sample = {
            'stamp_sec': int(msg.header.stamp.sec),
            'stamp_nanosec': int(msg.header.stamp.nanosec),
            'detections': detections,
        }

        with self._capture_lock:
            if not self._capture_active:
                return
            self._capture_buffer.append(sample)
            if len(self._capture_buffer) >= self.capture_depth:
                self._capture_active = False
                self._capture_ready.set()

    def _keyboard_loop(self) -> None:
        """Wait for operator triggers and manage one capture session at a time.

        The loop only runs when stdin is attached to a terminal so the prompt is
        visible and the user can interactively type a tag description.
        """
        while rclpy.ok() and not self._shutdown_event.is_set():
            try:
                sys.stdout.flush()
                command = input('\nPress Enter to capture the next tag, or q then Enter to quit: ').strip().lower()
            except EOFError:
                self.get_logger().warn('stdin closed; keyboard capture loop will stop.')
                return
            except Exception as exc:
                self.get_logger().warn(f'Keyboard input error: {exc}')
                return

            if command == 'q':
                self.get_logger().info('Keyboard capture loop requested shutdown.')
                self._shutdown_event.set()
                return

            self._run_capture_session()

    def _run_capture_session(self) -> None:
        """Capture a fixed window of detections and persist the stable result.

        Returns:
            None: The method only produces side effects (logging and persistence).
        """
        with self._capture_lock:
            self._capture_buffer.clear()
            self._capture_ready.clear()
            self._capture_active = True

        self.get_logger().info(
            f'Capture armed. Buffering the next {self.capture_depth} detection messages.'
        )

        if not self._capture_ready.wait(timeout=self.capture_wait_timeout_sec):
            with self._capture_lock:
                self._capture_active = False
            self.get_logger().warn(
                f'Capture timed out after {self.capture_wait_timeout_sec:.1f} seconds before the buffer filled.'
            )
            return

        with self._capture_lock:
            buffered_samples = list(self._capture_buffer)
            self._capture_active = False

        stable_pose = self._evaluate_stable_pose(buffered_samples)
        if stable_pose is None:
            self.get_logger().warn('Buffered detections were not stable enough; capture discarded.')
            return

        description = self._prompt_for_description(str(stable_pose['tag_frame']))
        if description is None:
            self.get_logger().info('Capture cancelled before saving.')
            return

        self._save_tag_entry(stable_pose, description)

    def _extract_detections(self, msg: AprilTagDetectionArray) -> List[Dict[str, object]]:
        """Convert a detection array message into a compact Python structure.

        Args:
            msg: Raw detection array from apriltag_ros.

        Returns:
            List[Dict[str, object]]: Detection records with family, id, and
            quality metadata.
        """
        detections: List[Dict[str, object]] = []
        for detection in msg.detections:
            family = str(detection.family).strip()
            tag_id = int(detection.id)
            detections.append(
                {
                    'family': family,
                    'id': tag_id,
                    'hamming': int(detection.hamming),
                    'decision_margin': float(detection.decision_margin),
                    'tag_frame': self._tag_frame_name(family, tag_id),
                }
            )
        return detections

    def _evaluate_stable_pose(self, buffered_samples: List[Dict[str, object]]) -> Optional[Dict[str, object]]:
        """Estimate a stable map-frame pose from a buffered capture window.

        Args:
            buffered_samples: Buffered detection samples gathered during capture.

        Returns:
            Optional[Dict[str, object]]: Stable pose metadata when a tag can be
            resolved consistently across the capture window, otherwise None.
        """
        if len(buffered_samples) < self.min_detection_hits:
            self.get_logger().warn(
                f'Need at least {self.min_detection_hits} buffered samples; got {len(buffered_samples)}.'
            )
            return None

        counter: Counter[Tuple[str, int]] = Counter()
        records_by_key: Dict[Tuple[str, int], List[Dict[str, object]]] = {}
        for sample in buffered_samples:
            for detection in sample['detections']:
                key = (str(detection['family']), int(detection['id']))
                counter[key] += 1
                records_by_key.setdefault(key, []).append(detection)

        if not counter:
            return None

        candidate_key, candidate_hits = counter.most_common(1)[0]
        if candidate_hits < self.min_detection_hits:
            self.get_logger().warn(
                f'Most common tag {candidate_key[0]}:{candidate_key[1]} only appeared {candidate_hits} times.'
            )
            return None

        positions: List[Tuple[float, float, float]] = []
        orientation: Optional[TransformStamped] = None
        latest_stamp_sec = 0
        latest_stamp_nanosec = 0

        for sample in buffered_samples:
            if not any(
                str(detection['family']) == candidate_key[0] and int(detection['id']) == candidate_key[1]
                for detection in sample['detections']
            ):
                continue

            transform = self._lookup_transform(candidate_key[0], candidate_key[1])
            if transform is None:
                continue

            positions.append(
                (
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y),
                    float(transform.transform.translation.z),
                )
            )
            orientation = transform
            latest_stamp_sec = int(transform.header.stamp.sec)
            latest_stamp_nanosec = int(transform.header.stamp.nanosec)

        if len(positions) < self.min_detection_hits:
            self.get_logger().warn(
                f'Could not resolve enough transforms for {candidate_key[0]}:{candidate_key[1]}.'
            )
            return None

        mean_position = (
            fmean(position[0] for position in positions),
            fmean(position[1] for position in positions),
            fmean(position[2] for position in positions),
        )
        max_delta = max(math.dist(position, mean_position) for position in positions)

        if max_delta > self.translation_threshold_m:
            self.get_logger().warn(
                f'Tag {candidate_key[0]}:{candidate_key[1]} moved by up to {max_delta:.3f} m; '
                f'threshold is {self.translation_threshold_m:.3f} m.'
            )
            return None

        if orientation is None:
            return None

        return {
            'tag_family': candidate_key[0],
            'tag_id': candidate_key[1],
            'tag_frame': self._tag_frame_name(candidate_key[0], candidate_key[1]),
            'map_frame': self.map_frame,
            'position': mean_position,
            'orientation': orientation.transform.rotation,
            'observations': len(positions),
            'candidate_hits': candidate_hits,
            'max_delta_m': max_delta,
            'stamp_sec': latest_stamp_sec,
            'stamp_nanosec': latest_stamp_nanosec,
            'sample_count': len(buffered_samples),
        }

    def _lookup_transform(self, family: str, tag_id: int) -> Optional[TransformStamped]:
        """Look up the map-frame transform for a detected AprilTag.

        Args:
            family: AprilTag family name reported by the detector.
            tag_id: AprilTag numeric identifier.

        Returns:
            Optional[TransformStamped]: The latest available map-to-tag transform
            when tf2 can resolve it, otherwise None.
        """
        tag_frame = self._tag_frame_name(family, tag_id)
        try:
            # Use the latest available transform because the robot is expected to
            # be stationary while the capture buffer is being filled.
            return self.tf_buffer.lookup_transform(self.map_frame, tag_frame, Time())
        except TransformException as exc:
            self.get_logger().warn(f'Could not resolve transform for {tag_frame}: {exc}')
            return None

    def _prompt_for_description(self, tag_frame: str) -> Optional[str]:
        """Prompt the operator for a tag description before saving.

        Args:
            tag_frame: Frame name of the tag being saved, used in the prompt.

        Returns:
            Optional[str]: The entered description, or None if the user cancels.
        """
        if not self._stdin_is_tty:
            self.get_logger().warn('No TTY available for description entry; skipping save.')
            return None

        try:
            description = input(f'Description for {tag_frame} [required]: ').strip()
        except EOFError:
            return None

        if not description:
            self.get_logger().info('Empty description entered; skipping save.')
            return None

        return description

    def _save_tag_entry(self, stable_pose: Dict[str, object], description: str) -> None:
        """Persist a stable tag pose into the map-specific JSON file.

        Args:
            stable_pose: Stable pose metadata returned by _evaluate_stable_pose.
            description: Human-readable description entered by the operator.
        """
        tags = self._load_tags()
        tag_frame = str(stable_pose['tag_frame'])
        existing_name = self._existing_tag_name(tags, tag_frame)
        name = existing_name or self._next_tag_name(tags)

        entry = {
            'name': name,
            'description': description,
            'map_name': self.map_name,
            'map_frame': str(stable_pose['map_frame']),
            'tag_frame': tag_frame,
            'tag_family': str(stable_pose['tag_family']),
            'tag_id': int(stable_pose['tag_id']),
            'x': float(stable_pose['position'][0]),
            'y': float(stable_pose['position'][1]),
            'z': float(stable_pose['position'][2]),
            'orientation': {
                'x': float(stable_pose['orientation'].x),
                'y': float(stable_pose['orientation'].y),
                'z': float(stable_pose['orientation'].z),
                'w': float(stable_pose['orientation'].w),
            },
            'observations': int(stable_pose['observations']),
            'candidate_hits': int(stable_pose['candidate_hits']),
            'translation_max_delta_m': float(stable_pose['max_delta_m']),
            'stamp': {
                'sec': int(stable_pose['stamp_sec']),
                'nanosec': int(stable_pose['stamp_nanosec']),
            },
        }

        tags = [existing for existing in tags if str(existing.get('tag_frame', '')).strip() != tag_frame]
        tags.append(entry)
        tags.sort(key=lambda item: str(item.get('name', '')))

        output_path = Path(self.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open('w', encoding='utf-8') as stream:
            json.dump(tags, stream, indent=2, sort_keys=False)
            stream.write('\n')

        self.get_logger().info(
            f'Saved tag "{name}" ({tag_frame}) at '
            f'({entry["x"]:.3f}, {entry["y"]:.3f}, {entry["z"]:.3f}) to {self.output_file}'
        )

    def _load_tags(self) -> List[Dict[str, object]]:
        """Load the existing tag list from the current output file.

        Returns:
            List[Dict[str, object]]: Parsed tag records, or an empty list if the
            file does not exist yet.
        """
        output_path = Path(self.output_file)
        if not output_path.exists():
            return []

        try:
            with output_path.open('r', encoding='utf-8') as stream:
                data = json.load(stream)
        except Exception as exc:
            self.get_logger().warn(f'Failed to read existing tag file {self.output_file}: {exc}')
            return []

        if isinstance(data, list):
            return data

        self.get_logger().warn(
            f'Expected tag file {self.output_file} to contain a list; got {type(data).__name__} instead.'
        )
        return []

    def _existing_tag_name(self, tags: List[Dict[str, object]], tag_frame: str) -> Optional[str]:
        """Return the stored name for a tag frame if it already exists.

        Args:
            tags: Existing tag records loaded from disk.
            tag_frame: Frame name that should be matched.

        Returns:
            Optional[str]: Existing tag name when a matching record is found.
        """
        for tag in tags:
            if str(tag.get('tag_frame', '')).strip() == tag_frame:
                candidate = str(tag.get('name', '')).strip()
                if candidate:
                    return candidate
        return None

    def _next_tag_name(self, tags: List[Dict[str, object]]) -> str:
        """Generate the next available tag name using the configured prefix.

        Args:
            tags: Existing tag records loaded from disk.

        Returns:
            str: Unique tag name such as tag_001 or tag_002.
        """
        existing = {str(tag.get('name', '')).strip() for tag in tags if str(tag.get('name', '')).strip()}
        index = 1
        while True:
            candidate = f'{self.name_prefix}_{index:03d}'
            if candidate not in existing:
                return candidate
            index += 1

    def _default_output_dir(self) -> Path:
        """Resolve the preferred storage directory for the active map.

        Returns:
            Path: Directory under map_tools/maps/<map_name> in the source tree
            when available, otherwise in the installed package share.
        """
        package_share = Path(get_package_share_directory('map_tools'))
        workspace_root = package_share.parent.parent.parent.parent
        src_maps_dir = workspace_root / 'src' / 'map_tools' / 'maps'

        if src_maps_dir.exists():
            return src_maps_dir / self._resolved_map_name()

        return package_share / 'maps' / self._resolved_map_name()

    def _resolved_map_name(self) -> str:
        """Return a filesystem-safe version of the active map name.

        Returns:
            str: Sanitized map name suitable for directory and file names.
        """
        candidate = self.map_name.strip() if self.map_name else ''
        if not candidate:
            candidate = 'default_map'
        return candidate.replace(' ', '_').replace('/', '_')

    def _resolve_output_file(self, raw_output_file: str) -> str:
        """Resolve the persistence target for collected tags.

        Args:
            raw_output_file: Optional explicit output file path.

        Returns:
            str: Absolute output path used for tag persistence.
        """
        if raw_output_file.strip():
            return os.path.expanduser(os.path.expandvars(raw_output_file.strip()))

        output_dir = self._default_output_dir()
        return str(output_dir / 'tags.json')

    def _tag_frame_name(self, family: str, tag_id: int) -> str:
        """Build the default tf frame name used by apriltag_ros.

        Args:
            family: AprilTag family name.
            tag_id: Numeric AprilTag identifier.

        Returns:
            str: Frame name such as tag36h11:5.
        """
        return f'tag{family}:{tag_id}'


def main(args: Optional[List[str]] = None) -> None:
    """Run the AprilTag tag collector node.

    Args:
        args: Optional ROS arguments forwarded to rclpy.init().

    Raises:
        KeyboardInterrupt: Propagated by the outer spin loop when the user stops
        the process.
    """
    rclpy.init(args=args)
    node = ApriltagTagCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()