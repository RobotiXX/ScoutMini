"""Assign stable IDs and bearings to 2D people detections."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from .track_schema import (
    Track2D,
    bearing_from_equirectangular,
    elevation_from_equirectangular,
    make_image_markers,
    make_people_frame,
    parse_people_frame,
)


class PeopleTracker(Node):
    def __init__(self) -> None:
        super().__init__('people_tracker')
        self.declare_parameter('detections_topic', '/people/detections_2d')
        self.declare_parameter('tracks_topic', '/people/tracks_2d')
        self.declare_parameter('marker_topic', '/people/tracks_2d/markers')
        self.declare_parameter('max_match_pixel_distance', 120.0)
        self.declare_parameter('track_timeout_sec', 0.75)
        self.declare_parameter('frame_id', 'camera_360')
        self.declare_parameter('equirectangular_center_x_norm', 0.5)
        self.declare_parameter('bearing_offset_rad', 0.0)

        detections_topic = str(self.get_parameter('detections_topic').value)
        tracks_topic = str(self.get_parameter('tracks_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        self.next_track_id = 1
        self.tracks: Dict[int, Dict[str, object]] = {}
        self.sub = self.create_subscription(String, detections_topic, self._callback, 10)
        self.pub = self.create_publisher(String, tracks_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.get_logger().info(f'Tracking detections from {detections_topic} to {tracks_topic}')

    def _callback(self, msg: String) -> None:
        try:
            frame = parse_people_frame(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Invalid detections JSON: {exc}')
            return

        stamp = self.get_clock().now().to_msg()
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        timeout = float(self.get_parameter('track_timeout_sec').value)
        self._drop_stale(now_sec, timeout)

        tracks: List[Track2D] = []
        assigned = set()
        for detection in frame.people:
            bbox = [float(v) for v in detection.get('bbox_xyxy', [0.0, 0.0, 0.0, 0.0])]
            width = int(detection.get('image_width', 0))
            height = int(detection.get('image_height', 0))
            center = self._center(bbox)
            track_id = self._resolve_track_id(detection, center, assigned)
            assigned.add(track_id)

            self.tracks[track_id] = {
                'center': center,
                'last_seen': now_sec,
                'bbox': bbox,
            }
            bearing = bearing_from_equirectangular(
                center[0],
                width,
                float(self.get_parameter('equirectangular_center_x_norm').value),
                float(self.get_parameter('bearing_offset_rad').value),
            )
            elevation = elevation_from_equirectangular(center[1], height)
            tracks.append(
                Track2D(
                    track_id=track_id,
                    label=str(detection.get('label', 'person')),
                    confidence=float(detection.get('confidence', 0.0)),
                    bbox_xyxy=bbox,
                    image_width=width,
                    image_height=height,
                    bearing_rad=bearing,
                    elevation_rad=elevation,
                    age_sec=0.0,
                    source=str(detection.get('source', 'tracker')),
                )
            )

        frame_id = str(self.get_parameter('frame_id').value) or frame.frame_id
        out_frame = make_people_frame(stamp, frame_id, tracks)
        out = String()
        out.data = out_frame.to_json()
        self.pub.publish(out)
        self.marker_pub.publish(make_image_markers(stamp, frame_id, out_frame.people, 'people_tracks_2d'))

    def _resolve_track_id(
        self,
        detection: Dict[str, object],
        center: Tuple[float, float],
        assigned: set,
    ) -> int:
        incoming_id = detection.get('track_id')
        if incoming_id is not None:
            try:
                return int(incoming_id)
            except (TypeError, ValueError):
                pass

        match_id = self._nearest_track(center, assigned)
        if match_id is not None:
            return match_id

        track_id = self.next_track_id
        self.next_track_id += 1
        return track_id

    def _nearest_track(self, center: Tuple[float, float], assigned: set) -> Optional[int]:
        max_distance = float(self.get_parameter('max_match_pixel_distance').value)
        best_id = None
        best_distance = max_distance
        for track_id, state in self.tracks.items():
            if track_id in assigned:
                continue
            previous = state.get('center', (0.0, 0.0))
            distance = math.hypot(center[0] - previous[0], center[1] - previous[1])
            if distance < best_distance:
                best_distance = distance
                best_id = track_id
        return best_id

    def _drop_stale(self, now_sec: float, timeout: float) -> None:
        stale_ids = [
            track_id
            for track_id, state in self.tracks.items()
            if now_sec - float(state.get('last_seen', 0.0)) > timeout
        ]
        for track_id in stale_ids:
            self.tracks.pop(track_id, None)

    @staticmethod
    def _center(bbox: List[float]) -> Tuple[float, float]:
        x1, y1, x2, y2 = bbox
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))


def main() -> None:
    rclpy.init()
    node = PeopleTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
