"""Transform projected people between ROS frames using TF."""

from __future__ import annotations

import math
from typing import Any, Dict

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from .adascore_people_adapter_node import _person_yaw
from .track_schema import make_people_frame, make_projected_markers, parse_people_frame


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _rotate_vector(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float]:
    # Quaternion-vector multiplication without depending on tf2_geometry_msgs.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def transform_person(person: Dict[str, Any], transform: TransformStamped) -> Dict[str, Any]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    x = float(person.get('x', 0.0))
    y = float(person.get('y', 0.0))
    z = float(person.get('z', 0.0))
    rx, ry, rz = _rotate_vector(x, y, z, rotation.x, rotation.y, rotation.z, rotation.w)

    vx = float(person.get('vx', 0.0))
    vy = float(person.get('vy', 0.0))
    vz = float(person.get('vz', 0.0))
    rvx, rvy, rvz = _rotate_vector(vx, vy, vz, rotation.x, rotation.y, rotation.z, rotation.w)

    out = dict(person)
    out['x'] = rx + translation.x
    out['y'] = ry + translation.y
    out['z'] = rz + translation.z
    out['vx'] = rvx
    out['vy'] = rvy
    out['vz'] = rvz
    out['yaw_rad'] = _person_yaw(person) + _yaw_from_quaternion(
        rotation.x, rotation.y, rotation.z, rotation.w
    )
    out['source'] = 'people_frame_transform'
    return out


class PeopleFrameTransform(Node):
    def __init__(self) -> None:
        super().__init__('people_frame_transform')
        self.declare_parameter('input_topic', '/people/projected')
        self.declare_parameter('output_topic', '/people/projected_map')
        self.declare_parameter('marker_topic', '/people/projected_map/markers')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('use_latest_tf', True)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(String, input_topic, self._callback, 10)
        self.pub = self.create_publisher(String, output_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.get_logger().info(f'Transforming {input_topic} to {output_topic}')

    def _callback(self, msg: String) -> None:
        try:
            frame = parse_people_frame(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Invalid projected people JSON: {exc}')
            return

        target_frame = str(self.get_parameter('target_frame').value)
        if frame.frame_id == target_frame:
            out = msg
            self.pub.publish(out)
            self.marker_pub.publish(
                make_projected_markers(self.get_clock().now().to_msg(), target_frame, frame.people)
            )
            return

        try:
            transform = self._lookup_transform(target_frame, frame.frame_id, frame.stamp_sec)
        except TransformException as exc:
            self.get_logger().warn(
                f'No TF from {frame.frame_id!r} to {target_frame!r}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        now_msg = self.get_clock().now().to_msg()
        people = [transform_person(person, transform) for person in frame.people]
        out_frame = make_people_frame(now_msg, target_frame, people)
        out = String()
        out.data = out_frame.to_json()
        self.pub.publish(out)
        self.marker_pub.publish(make_projected_markers(now_msg, target_frame, out_frame.people))

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp_sec: float) -> TransformStamped:
        timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        use_latest = bool(self.get_parameter('use_latest_tf').value)
        stamp = Time() if use_latest else Time(seconds=stamp_sec)
        return self.tf_buffer.lookup_transform(target_frame, source_frame, stamp, timeout)


def main() -> None:
    rclpy.init()
    node = PeopleFrameTransform()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
