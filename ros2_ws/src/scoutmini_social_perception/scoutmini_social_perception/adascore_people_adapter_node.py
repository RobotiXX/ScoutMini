"""Boundary adapter for AdaSCoRe people-topic integration."""

from __future__ import annotations

import importlib
import json
import math
from typing import Any, Dict, Optional

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Header
from std_msgs.msg import String

from .track_schema import parse_people_frame


def _load_people_message_types() -> tuple[Optional[type], Optional[type]]:
    try:
        people_msgs = importlib.import_module('people_msgs.msg')
    except ImportError:
        return None, None
    return getattr(people_msgs, 'People', None), getattr(people_msgs, 'Person', None)


def _time_from_seconds(stamp_sec: float) -> Time:
    stamp = Time()
    stamp.sec = int(stamp_sec)
    stamp.nanosec = int((stamp_sec - stamp.sec) * 1e9)
    return stamp


def _person_yaw(person: Dict[str, Any]) -> float:
    explicit_yaw = person.get('yaw_rad')
    if explicit_yaw is not None:
        return float(explicit_yaw)
    vx = float(person.get('vx', 0.0))
    vy = float(person.get('vy', 0.0))
    if abs(vx) > 1e-3 or abs(vy) > 1e-3:
        return math.atan2(vy, vx)
    return float(person.get('bearing_rad', 0.0))


class AdaScorePeopleAdapter(Node):
    """Republish projected people behind an explicit disabled-by-default gate."""

    def __init__(self) -> None:
        super().__init__('adascore_people_adapter')
        self.declare_parameter('projected_topic', '/people/projected')
        self.declare_parameter('adascore_people_topic', '/people')
        self.declare_parameter('debug_topic', '/adascore/people_debug')
        self.declare_parameter('enabled', False)
        self.declare_parameter('output_message_type', 'json_debug')
        self.declare_parameter('adascore_frame_id', 'map')
        self.declare_parameter('require_frame_match', True)

        projected_topic = str(self.get_parameter('projected_topic').value)
        adascore_people_topic = str(self.get_parameter('adascore_people_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.output_message_type = str(self.get_parameter('output_message_type').value)
        self.adascore_frame_id = str(self.get_parameter('adascore_frame_id').value)
        self.require_frame_match = bool(self.get_parameter('require_frame_match').value)
        self.PeopleMsg, self.PersonMsg = _load_people_message_types()

        self.sub = self.create_subscription(String, projected_topic, self._callback, 10)
        self.debug_pub = self.create_publisher(String, debug_topic, 10)
        self.json_pub = None
        self.people_pub = None

        if self.output_message_type == 'json_debug':
            self.json_pub = self.create_publisher(String, adascore_people_topic, 10)
        elif self.output_message_type == 'people_msgs':
            if self.PeopleMsg is None or self.PersonMsg is None:
                self.get_logger().error(
                    'output_message_type=people_msgs requested, but people_msgs is not installed. '
                    'Install wg-perception/people on the ros2 branch before enabling AdaSCoRe output.'
                )
            else:
                self.people_pub = self.create_publisher(self.PeopleMsg, adascore_people_topic, 10)
        else:
            self.get_logger().error(
                f'Unsupported output_message_type={self.output_message_type!r}; expected json_debug or people_msgs.'
            )

        self.get_logger().warn(
            'AdaSCoRe adapter '
            f'enabled={self.enabled}, output_message_type={self.output_message_type}, '
            f'topic={adascore_people_topic}, required_frame={self.adascore_frame_id}.'
        )

    def _callback(self, msg: String) -> None:
        try:
            frame = parse_people_frame(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Invalid projected people JSON: {exc}')
            return

        debug = String()
        debug.data = json.dumps(
            {
                'enabled': self.enabled,
                'output_message_type': self.output_message_type,
                'schema': frame.schema,
                'frame_id': frame.frame_id,
                'adascore_frame_id': self.adascore_frame_id,
                'people_count': len(frame.people),
                'people_msgs_available': self.PeopleMsg is not None and self.PersonMsg is not None,
            },
            separators=(',', ':'),
            sort_keys=True,
        )
        self.debug_pub.publish(debug)

        if not self.enabled:
            return

        if self.output_message_type == 'json_debug' and self.json_pub is not None:
            out = String()
            out.data = msg.data
            self.json_pub.publish(out)
            return

        if self.output_message_type != 'people_msgs' or self.people_pub is None:
            return

        if self.require_frame_match and frame.frame_id != self.adascore_frame_id:
            self.get_logger().warn(
                f'Dropping AdaSCoRe people output because frame_id={frame.frame_id!r} '
                f'does not match required {self.adascore_frame_id!r}. '
                'Publish projected people in the AdaSCoRe/Nav2 controller frame before enabling this output.',
                throttle_duration_sec=5.0,
            )
            return

        self.people_pub.publish(self._to_people_msg(frame.stamp_sec, frame.frame_id, frame.people))

    def _to_people_msg(self, stamp_sec: float, frame_id: str, people: list[Dict[str, Any]]) -> Any:
        assert self.PeopleMsg is not None
        assert self.PersonMsg is not None

        msg = self.PeopleMsg()
        msg.header = Header()
        msg.header.stamp = _time_from_seconds(stamp_sec)
        msg.header.frame_id = frame_id
        msg.people = []

        for person in people:
            out = self.PersonMsg()
            track_id = int(person.get('track_id', len(msg.people)))
            out.name = f'person_{track_id}'
            out.position = Point()
            out.position.x = float(person.get('x', 0.0))
            out.position.y = float(person.get('y', 0.0))
            out.position.z = _person_yaw(person)
            out.velocity = Point()
            out.velocity.x = float(person.get('vx', 0.0))
            out.velocity.y = float(person.get('vy', 0.0))
            out.velocity.z = float(person.get('wz', 0.0))
            out.reliability = float(person.get('confidence', 0.0))
            out.tagnames = ['track_id', 'source', 'range_source']
            out.tags = [
                str(track_id),
                str(person.get('source', 'scoutmini_social_perception')),
                str(person.get('range_source', 'unknown')),
            ]
            msg.people.append(out)
        return msg


def main() -> None:
    rclpy.init()
    node = AdaScorePeopleAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
