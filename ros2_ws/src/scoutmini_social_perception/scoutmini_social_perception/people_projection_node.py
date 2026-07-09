"""Project tracked 2D people into a robot-relative ground plane."""

from __future__ import annotations

import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from .track_schema import ProjectedPerson, make_people_frame, make_projected_markers, parse_people_frame


def estimate_person_range(
    track: Dict[str, object],
    mode: str,
    fixed_range_m: float,
    assumed_person_height_m: float,
) -> tuple[float, str]:
    fixed_range = max(0.1, float(fixed_range_m))
    if mode == 'person_height_ground_plane':
        bbox = [float(v) for v in track.get('bbox_xyxy', [0.0, 0.0, 0.0, 0.0])]
        image_height = max(1, int(track.get('image_height', 1)))
        bbox_height = max(1.0, bbox[3] - bbox[1])
        angular_height = (bbox_height / float(image_height)) * math.pi
        assumed_height = max(0.5, float(assumed_person_height_m))
        estimated_range = assumed_height / max(0.05, angular_height)
        return min(max(estimated_range, 0.2), 12.0), 'person_height_ground_plane'
    return fixed_range, 'fixed_distance_debug'


class PeopleProjection(Node):
    def __init__(self) -> None:
        super().__init__('people_projection')
        self.declare_parameter('tracks_topic', '/people/tracks_2d')
        self.declare_parameter('projected_topic', '/people/projected')
        self.declare_parameter('marker_topic', '/people/projected/markers')
        self.declare_parameter('source_frame', 'camera_360')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('fixed_range_m', 3.0)
        self.declare_parameter('range_mode', 'fixed_distance_debug')
        self.declare_parameter('assumed_person_height_m', 1.7)
        self.declare_parameter('stale_timeout_sec', 0.75)

        tracks_topic = str(self.get_parameter('tracks_topic').value)
        projected_topic = str(self.get_parameter('projected_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        self.previous_positions: Dict[int, Dict[str, float]] = {}
        self.sub = self.create_subscription(String, tracks_topic, self._callback, 10)
        self.pub = self.create_publisher(String, projected_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.get_logger().info(f'Projecting {tracks_topic} to {projected_topic}')

    def _callback(self, msg: String) -> None:
        try:
            frame = parse_people_frame(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Invalid tracks JSON: {exc}')
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()
        now_sec = now.nanoseconds * 1e-9
        projected: List[ProjectedPerson] = []
        for track in frame.people:
            track_id = int(track.get('track_id', 0))
            bearing = float(track.get('bearing_rad', 0.0))
            range_m, range_source = self._estimate_range(track)
            x = range_m * math.cos(bearing)
            y = range_m * math.sin(bearing)
            vx, vy = self._estimate_velocity(track_id, x, y, now_sec)
            projected.append(
                ProjectedPerson(
                    track_id=track_id,
                    label=str(track.get('label', 'person')),
                    confidence=float(track.get('confidence', 0.0)),
                    x=x,
                    y=y,
                    z=0.0,
                    range_m=range_m,
                    bearing_rad=bearing,
                    vx=vx,
                    vy=vy,
                    range_source=range_source,
                    source='people_projection',
                )
            )

        output_frame = str(self.get_parameter('output_frame').value)
        out_frame = make_people_frame(now_msg, output_frame, projected)
        out = String()
        out.data = out_frame.to_json()
        self.pub.publish(out)
        self.marker_pub.publish(make_projected_markers(now_msg, output_frame, out_frame.people))

    def _estimate_range(self, track: Dict[str, object]) -> tuple[float, str]:
        return estimate_person_range(
            track,
            str(self.get_parameter('range_mode').value),
            float(self.get_parameter('fixed_range_m').value),
            float(self.get_parameter('assumed_person_height_m').value),
        )

    def _estimate_velocity(self, track_id: int, x: float, y: float, now_sec: float) -> tuple[float, float]:
        previous = self.previous_positions.get(track_id)
        self.previous_positions[track_id] = {'x': x, 'y': y, 't': now_sec}
        if previous is None:
            return 0.0, 0.0
        dt = max(1e-3, now_sec - previous['t'])
        return (x - previous['x']) / dt, (y - previous['y']) / dt


def main() -> None:
    rclpy.init()
    node = PeopleProjection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
