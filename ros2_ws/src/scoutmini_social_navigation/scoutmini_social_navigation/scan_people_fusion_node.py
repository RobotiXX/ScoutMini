"""Fuse equirectangular people tracks with a planar LiDAR scan."""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from people_msgs.msg import People, Person
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray

from .fusion import (
    equirectangular_bearing,
    numeric_track_id,
    robust_scan_range,
    rotate_vector_by_quaternion,
    transform_point,
)


class ScanPeopleFusion(Node):
    """Publish only tracks with a current, valid LiDAR range."""

    def __init__(self) -> None:
        super().__init__('scan_people_fusion')
        self.declare_parameter('tracks_topic', '/people/tracks_2d')
        self.declare_parameter('camera_info_topic', '/equirectangular/image/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('people_topic', '/adascore/shadow/people')
        self.declare_parameter('diagnostics_topic', '/people/fusion_diagnostics')
        self.declare_parameter('camera_frame', '360_link')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('fallback_image_width', 1920)
        self.declare_parameter('equirectangular_center_x_norm', 0.5)
        self.declare_parameter('bearing_offset_rad', 0.0)
        self.declare_parameter('max_scan_age_sec', 0.25)
        self.declare_parameter('min_window_rad', 0.025)
        self.declare_parameter('max_window_rad', 0.18)
        self.declare_parameter('window_scale', 0.30)
        self.declare_parameter('min_range_m', 0.35)
        self.declare_parameter('max_range_m', 10.0)
        self.declare_parameter('min_scan_samples', 2)
        self.declare_parameter('track_timeout_sec', 1.0)
        self.declare_parameter('max_range_rate_mps', 4.0)
        self.declare_parameter('velocity_alpha', 0.35)

        self._scan: Optional[LaserScan] = None
        self._image_width = int(self.get_parameter('fallback_image_width').value)
        self._states: Dict[str, Dict[str, float]] = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter('tracks_topic').value),
            self._tracks_callback,
            qos_profile_sensor_data,
        )
        self._people_pub = self.create_publisher(
            People,
            str(self.get_parameter('people_topic').value),
            10,
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )
        self.get_logger().info(
            f'Validated people output: {self.get_parameter("people_topic").value}'
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan = msg

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if msg.width > 0:
            self._image_width = int(msg.width)

    def _tracks_callback(self, msg: Detection2DArray) -> None:
        output = People()
        output.header = msg.header
        output.header.frame_id = str(self.get_parameter('output_frame').value)
        rejected = 0

        scan = self._scan
        if scan is None or not self._scan_is_current(msg, scan):
            self._publish_diagnostics(msg, 0, len(msg.detections), 'scan unavailable or stale')
            self._people_pub.publish(output)
            return

        camera_frame = msg.header.frame_id or str(self.get_parameter('camera_frame').value)
        try:
            scan_from_camera = self._lookup(scan.header.frame_id, camera_frame, msg.header.stamp)
            output_from_scan = self._lookup(
                output.header.frame_id,
                scan.header.frame_id,
                msg.header.stamp,
            )
        except TransformException as exc:
            self.get_logger().warning(f'People fusion transform unavailable: {exc}')
            self._publish_diagnostics(msg, 0, len(msg.detections), 'transform unavailable')
            self._people_pub.publish(output)
            return

        for detection in msg.detections:
            person = self._fuse_detection(
                detection,
                msg,
                scan,
                scan_from_camera,
                output_from_scan,
            )
            if person is None:
                rejected += 1
            else:
                output.people.append(person)

        self._drop_stale(self._stamp_sec(msg.header.stamp))
        self._people_pub.publish(output)
        self._publish_diagnostics(msg, len(output.people), rejected, 'tracking')

    def _fuse_detection(self, detection, tracks, scan, scan_from_camera, output_from_scan):
        width = max(1, self._image_width)
        bearing_camera = equirectangular_bearing(
            detection.bbox.center.position.x,
            width,
            float(self.get_parameter('equirectangular_center_x_norm').value),
            float(self.get_parameter('bearing_offset_rad').value),
        )
        scan_quaternion = self._quaternion(scan_from_camera)
        direction_scan = rotate_vector_by_quaternion(
            (math.cos(bearing_camera), math.sin(bearing_camera), 0.0),
            scan_quaternion,
        )
        bearing_scan = math.atan2(direction_scan[1], direction_scan[0])
        half_width = self._scan_half_width(detection, width)
        range_m = robust_scan_range(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            bearing_scan,
            half_width,
            max(float(scan.range_min), float(self.get_parameter('min_range_m').value)),
            min(float(scan.range_max), float(self.get_parameter('max_range_m').value)),
            int(self.get_parameter('min_scan_samples').value),
        )
        if range_m is None:
            return None

        point_scan = (
            range_m * math.cos(bearing_scan),
            range_m * math.sin(bearing_scan),
            0.0,
        )
        point_output = transform_point(
            point_scan,
            self._translation(output_from_scan),
            self._quaternion(output_from_scan),
        )
        stamp_sec = self._stamp_sec(tracks.header.stamp)
        motion = self._motion(detection.id, point_output, stamp_sec)
        if motion is None:
            return None
        velocity_x, velocity_y, yaw = motion

        person = Person()
        person.name = detection.id
        person.position.x = point_output[0]
        person.position.y = point_output[1]
        person.position.z = yaw
        person.velocity.x = velocity_x
        person.velocity.y = velocity_y
        person.velocity.z = 0.0
        person.reliability = (
            detection.results[0].hypothesis.score if detection.results else 0.0
        )
        person.tagnames = ['track_id', 'group_id', 'range_source']
        person.tags = [str(numeric_track_id(detection.id)), '-1', 'velodyne_scan']
        return person

    def _motion(
        self,
        track_id: str,
        position: Tuple[float, float, float],
        stamp_sec: float,
    ) -> Optional[Tuple[float, float, float]]:
        previous = self._states.get(track_id)
        if previous is None or stamp_sec <= previous['stamp']:
            vx = vy = 0.0
        else:
            dt = stamp_sec - previous['stamp']
            raw_vx = (position[0] - previous['x']) / dt
            raw_vy = (position[1] - previous['y']) / dt
            if math.hypot(raw_vx, raw_vy) > float(
                self.get_parameter('max_range_rate_mps').value
            ):
                return None
            alpha = float(self.get_parameter('velocity_alpha').value)
            vx = alpha * raw_vx + (1.0 - alpha) * previous['vx']
            vy = alpha * raw_vy + (1.0 - alpha) * previous['vy']
        speed = math.hypot(vx, vy)
        if speed > 0.09:
            yaw = math.atan2(vy, vx)
        elif previous:
            yaw = previous.get('yaw', 0.0)
        else:
            yaw = 0.0
        self._states[track_id] = {
            'x': position[0],
            'y': position[1],
            'stamp': stamp_sec,
            'vx': vx,
            'vy': vy,
            'yaw': yaw,
        }
        return vx, vy, yaw

    def _scan_is_current(self, tracks: Detection2DArray, scan: LaserScan) -> bool:
        track_stamp = self._stamp_sec(tracks.header.stamp)
        scan_stamp = self._stamp_sec(scan.header.stamp)
        if track_stamp <= 0.0 or scan_stamp <= 0.0:
            return True
        return abs(track_stamp - scan_stamp) <= float(
            self.get_parameter('max_scan_age_sec').value
        )

    def _scan_half_width(self, detection: Detection2D, width: int) -> float:
        box_radians = detection.bbox.size_x / width * 2.0 * math.pi
        value = box_radians * float(self.get_parameter('window_scale').value)
        return min(
            max(value, float(self.get_parameter('min_window_rad').value)),
            float(self.get_parameter('max_window_rad').value),
        )

    def _lookup(self, target: str, source: str, stamp) -> object:
        if target == source:
            from geometry_msgs.msg import TransformStamped

            identity = TransformStamped()
            identity.header.frame_id = target
            identity.child_frame_id = source
            identity.transform.rotation.w = 1.0
            return identity
        return self._tf_buffer.lookup_transform(
            target,
            source,
            Time.from_msg(stamp),
            timeout=Duration(seconds=0.10),
        )

    def _drop_stale(self, stamp_sec: float) -> None:
        timeout = float(self.get_parameter('track_timeout_sec').value)
        self._states = {
            track_id: state
            for track_id, state in self._states.items()
            if stamp_sec - state['stamp'] <= timeout
        }

    def _publish_diagnostics(self, msg, accepted: int, rejected: int, message: str):
        array = DiagnosticArray()
        array.header = msg.header
        status = DiagnosticStatus()
        status.name = 'scoutmini_social_navigation/scan_people_fusion'
        status.hardware_id = 'insta360_x4+velodyne_vlp16'
        status.level = DiagnosticStatus.OK if accepted or not rejected else DiagnosticStatus.WARN
        status.message = message
        status.values = [
            KeyValue(key='accepted_tracks', value=str(accepted)),
            KeyValue(key='rejected_tracks', value=str(rejected)),
            KeyValue(key='output_frame', value=str(self.get_parameter('output_frame').value)),
        ]
        array.status.append(status)
        self._diagnostics_pub.publish(array)

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _quaternion(transform) -> Tuple[float, float, float, float]:
        rotation = transform.transform.rotation
        return rotation.x, rotation.y, rotation.z, rotation.w

    @staticmethod
    def _translation(transform) -> Tuple[float, float, float]:
        translation = transform.transform.translation
        return translation.x, translation.y, translation.z


def main() -> None:
    rclpy.init()
    node = ScanPeopleFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
