"""Exercise typed-track to validated-People fusion without robot hardware."""

import math
import time

from geometry_msgs.msg import TransformStamped
from people_msgs.msg import People
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray

from scoutmini_social_navigation.scan_people_fusion_node import (
    ScanPeopleFusion,
)


class FusionProbe(Node):
    """Publish deterministic inputs and capture validated people output."""

    def __init__(self) -> None:
        super().__init__('scan_people_fusion_smoke')
        self.people = None
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data,
        )
        self.camera_pub = self.create_publisher(
            CameraInfo,
            '/equirectangular/image/camera_info',
            qos_profile_sensor_data,
        )
        self.tracks_pub = self.create_publisher(
            Detection2DArray,
            '/people/tracks_2d',
            qos_profile_sensor_data,
        )
        self.create_subscription(
            People,
            '/adascore/shadow/people',
            self._people_callback,
            10,
        )
        self.tf_broadcaster = StaticTransformBroadcaster(self)

    def publish_transforms(self) -> None:
        """Publish identity transforms for the synthetic frame tree."""
        transforms = []
        for parent, child in [('odom', 'scan'), ('scan', '360_link')]:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.tf_broadcaster.sendTransform(transforms)

    def publish_inputs(self) -> None:
        """Publish a centered track with a two-meter LiDAR return."""
        stamp = self.get_clock().now().to_msg()

        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = '360_link'
        camera_info.width = 1920
        camera_info.height = 960
        self.camera_pub.publish(camera_info)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'scan'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = [math.inf] * 360
        for index in range(177, 184):
            scan.ranges[index] = 2.0
        self.scan_pub.publish(scan)

        tracks = Detection2DArray()
        tracks.header.stamp = stamp
        tracks.header.frame_id = '360_link'
        detection = Detection2D()
        detection.header = tracks.header
        detection.id = 'person_000042'
        detection.bbox.center.position.x = 960.0
        detection.bbox.center.position.y = 480.0
        detection.bbox.size_x = 120.0
        detection.bbox.size_y = 400.0
        tracks.detections.append(detection)
        self.tracks_pub.publish(tracks)

    def _people_callback(self, msg: People) -> None:
        if msg.people:
            self.people = msg


def main() -> None:
    """Run the bounded smoke test and fail if fusion is not valid."""
    rclpy.init()
    fusion = ScanPeopleFusion()
    probe = FusionProbe()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(fusion)
    executor.add_node(probe)
    deadline = time.monotonic() + 8.0
    next_publish = 0.0
    try:
        probe.publish_transforms()
        while time.monotonic() < deadline and probe.people is None:
            now = time.monotonic()
            if now >= next_publish:
                probe.publish_inputs()
                next_publish = now + 0.20
            executor.spin_once(timeout_sec=0.05)
        assert probe.people is not None, 'No validated People output received'
        person = probe.people.people[0]
        assert person.name == 'person_000042'
        assert abs(person.position.x - 2.0) < 0.15
        assert abs(person.position.y) < 0.15
        assert person.tags[:2] == ['42', '-1']
        print(
            'fusion smoke passed: '
            f'{person.name} at ({person.position.x:.2f}, '
            f'{person.position.y:.2f}) m in {probe.people.header.frame_id}'
        )
    finally:
        executor.remove_node(probe)
        executor.remove_node(fusion)
        probe.destroy_node()
        fusion.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
