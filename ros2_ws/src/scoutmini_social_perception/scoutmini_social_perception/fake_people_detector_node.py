"""Publish deterministic fake people for pipeline and visualization tests."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from .track_schema import ProjectedPerson, make_people_frame, make_projected_markers


class FakePeopleDetector(Node):
    def __init__(self) -> None:
        super().__init__('fake_people_detector')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('output_topic', '/people/tracks')
        self.declare_parameter('marker_topic', '/people/markers')
        self.declare_parameter('frame_id', 'base_link')

        self.frame_id = self.get_parameter('frame_id').value
        publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        output_topic = str(self.get_parameter('output_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        self.publisher = self.create_publisher(String, output_topic, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._tick)

        self.get_logger().info(
            f'Publishing fake people at {publish_rate_hz:.1f} Hz on {output_topic}'
        )

    def _tick(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        people = [
            ProjectedPerson(
                track_id=1,
                label='person',
                confidence=0.99,
                x=2.4 + 0.15 * math.sin(elapsed),
                y=0.8,
                z=0.0,
                range_m=math.hypot(2.4, 0.8),
                bearing_rad=math.atan2(0.8, 2.4),
                range_source='fake',
                source='fake_people_detector',
            ),
            ProjectedPerson(
                track_id=2,
                label='person',
                confidence=0.92,
                x=3.1,
                y=-0.9 + 0.25 * math.sin(elapsed * 0.5),
                z=0.0,
                range_m=math.hypot(3.1, -0.9),
                bearing_rad=math.atan2(-0.9, 3.1),
                range_source='fake',
                source='fake_people_detector',
            ),
        ]

        frame = make_people_frame(now.to_msg(), self.frame_id, people)
        msg = String()
        msg.data = frame.to_json()
        self.publisher.publish(msg)
        self.marker_publisher.publish(
            make_projected_markers(now.to_msg(), self.frame_id, frame.people, 'fake_people')
        )


def main() -> None:
    rclpy.init()
    node = FakePeopleDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
