#!/usr/bin/env python3
"""Filter LaserScan returns near configured doors for global costmap use."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class DoorSegment:
    name: str
    start: Tuple[float, float]
    end: Tuple[float, float]


class DoorScanFilter(Node):
    def __init__(self) -> None:
        super().__init__('door_scan_filter')
        self.declare_parameter('map_name', 'fuse_3rd')
        self.declare_parameter('doors_file', '')
        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_global_filtered')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('filter_radius', 0.2)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.filter_radius = float(self.get_parameter('filter_radius').value)
        self.filter_radius_sq = self.filter_radius * self.filter_radius
        self.doors = self._load_doors()
        self.scan_count = 0
        self.publish_count = 0
        self.last_removed_count = 0
        self.missing_tf_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter('output_scan_topic').value),
            10,
        )
        self.sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter('input_scan_topic').value),
            self._scan_cb,
            qos_profile_sensor_data,
        )
        self.status_timer = self.create_timer(2.0, self._status_cb)
        self.get_logger().info(
            f'Filtering {len(self.doors)} door segment(s) from {self.get_parameter("input_scan_topic").value} '
            f'to {self.get_parameter("output_scan_topic").value} within {self.filter_radius:.2f} m'
        )

    def _default_doors_file(self) -> Path:
        map_name = str(self.get_parameter('map_name').value)
        map_tools_share = Path(get_package_share_directory('map_tools'))
        return map_tools_share / 'maps' / map_name / 'doors.json'

    def _load_doors(self) -> List[DoorSegment]:
        doors_file_param = str(self.get_parameter('doors_file').value)
        doors_file = Path(doors_file_param) if doors_file_param else self._default_doors_file()
        try:
            data = json.loads(doors_file.read_text(encoding='utf-8'))
        except Exception as exc:
            self.get_logger().error(f'Failed to load door config {doors_file}: {exc}')
            return []

        doors: List[DoorSegment] = []
        for item in data.get('doors', []):
            route_gate = item.get('route_gate', {})
            if not route_gate.get('pre_waypoint') or not route_gate.get('post_waypoint'):
                continue
            closed_pose = item.get('closed_pose', {})
            leaf = item.get('leaf', {})
            try:
                name = str(item['name'])
                hinge_x = float(closed_pose.get('x', 0.0))
                hinge_y = float(closed_pose.get('y', 0.0))
                yaw = float(closed_pose.get('yaw', 0.0))
                length = float(leaf.get('length', 1.0))
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warn(f'Skipping invalid door entry {item}: {exc}')
                continue
            end_x = hinge_x - math.sin(yaw) * length
            end_y = hinge_y + math.cos(yaw) * length
            doors.append(DoorSegment(name=name, start=(hinge_x, hinge_y), end=(end_x, end_y)))
        return doors

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan_count += 1
        if not self.doors:
            self.pub.publish(msg)
            self.publish_count += 1
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.missing_tf_count += 1
            self.get_logger().debug(f'No transform {msg.header.frame_id}->{self.target_frame}: {exc}')
            self.pub.publish(msg)
            self.publish_count += 1
            return

        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.intensities = msg.intensities
        filtered.ranges = list(msg.ranges)

        angle = msg.angle_min
        removed = 0
        for index, scan_range in enumerate(msg.ranges):
            if math.isfinite(scan_range) and msg.range_min <= scan_range <= msg.range_max:
                point = self._scan_point_to_target(scan_range, angle, transform)
                if self._near_any_door(point):
                    filtered.ranges[index] = math.inf
                    removed += 1
            angle += msg.angle_increment

        self.pub.publish(filtered)
        self.publish_count += 1
        self.last_removed_count = removed
        if removed:
            self.get_logger().debug(f'Removed {removed} scan point(s) near configured doors')


    def _status_cb(self) -> None:
        if self.scan_count == 0:
            self.get_logger().warn(
                f'Waiting for LaserScan on {self.get_parameter("input_scan_topic").value}; '
                f'no messages published yet on {self.get_parameter("output_scan_topic").value}'
            )
            return
        self.get_logger().info(
            f'Published {self.publish_count} filtered scan(s) to {self.get_parameter("output_scan_topic").value}; '
            f'last removed {self.last_removed_count} point(s); missing_tf={self.missing_tf_count}'
        )

    @staticmethod
    def _scan_point_to_target(
        scan_range: float,
        angle: float,
        transform: TransformStamped,
    ) -> Tuple[float, float]:
        local_x = scan_range * math.cos(angle)
        local_y = scan_range * math.sin(angle)
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        return (
            tx + cos_yaw * local_x - sin_yaw * local_y,
            ty + sin_yaw * local_x + cos_yaw * local_y,
        )

    def _near_any_door(self, point: Tuple[float, float]) -> bool:
        return any(self._point_segment_distance_sq(point, door.start, door.end) <= self.filter_radius_sq for door in self.doors)

    @staticmethod
    def _point_segment_distance_sq(
        point: Tuple[float, float],
        start: Tuple[float, float],
        end: Tuple[float, float],
    ) -> float:
        px, py = point
        ax, ay = start
        bx, by = end
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            ex = px - ax
            ey = py - ay
            return ex * ex + ey * ey
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        closest_x = ax + t * dx
        closest_y = ay + t * dy
        ex = px - closest_x
        ey = py - closest_y
        return ex * ex + ey * ey


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DoorScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
