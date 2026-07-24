"""py_trees behaviour for selecting and cycling simulated elevator doors."""

import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ElevatorDoor:
    def __init__(self, name: str, x: float, y: float, yaw: float, length: float):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.length = length

    @property
    def segment(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        return (
            (self.x, self.y),
            (
                self.x - math.sin(self.yaw) * self.length,
                self.y + math.cos(self.yaw) * self.length,
            ),
        )


class ElevatorSequence(py_trees.behaviour.Behaviour):
    """Choose an opening elevator, enter, await close/open, and exit."""

    def __init__(
        self,
        node: Node,
        doors_file: Path,
        waypoints: Dict[str, dict],
        scan_topic: str,
        target_frame: str,
        action_name: str,
        open_distance_margin: float = 0.1,
        open_ray_fraction: float = 0.5,
    ) -> None:
        super().__init__(name='ElevatorSequence')
        self.node = node
        self.waypoints = waypoints
        self.target_frame = target_frame
        self.open_distance_margin = open_distance_margin
        self.open_ray_fraction = open_ray_fraction
        self.doors = self._load_doors(doors_file)
        self.action_client = ActionClient(node, NavigateThroughPoses, action_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.scan_sub = node.create_subscription(
            LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data
        )
        # name -> (is_open, rays_beyond_closed_range, rays_intersecting_door, scan_sequence)
        self.observations: Dict[str, Tuple[bool, int, int, int]] = {}
        self.scan_sequence = 0
        self.phase_started_scan = 0
        self.phase = 'wait_open'
        self.started = False
        self.selected_door: Optional[ElevatorDoor] = None
        self.last_status_log_ns = 0
        self.goal_active = False
        self.goal_handle = None
        self.goal_destination = ''
        self.failed = False
        self.waiting_for_server_logged = False

    @staticmethod
    def _load_doors(doors_file: Path) -> List[ElevatorDoor]:
        with doors_file.open('r', encoding='utf-8') as stream:
            data = json.load(stream)
        doors = []
        for item in data.get('doors', []):
            name = str(item.get('name', ''))
            if not (name.startswith('elevator_') and name.endswith('_door')):
                continue
            pose = item.get('closed_pose', {})
            leaf = item.get('leaf', {})
            doors.append(ElevatorDoor(
                name=name,
                x=float(pose.get('x', 0.0)),
                y=float(pose.get('y', 0.0)),
                yaw=float(pose.get('yaw', 0.0)),
                length=float(leaf.get('length', 1.0)),
            ))
        return doors

    def _scan_cb(self, scan: LaserScan) -> None:
        if not self.started or self.phase == 'complete':
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, scan.header.frame_id, rclpy.time.Time()
            )
        except TransformException as exc:
            self.node.get_logger().debug(
                f'No transform {scan.header.frame_id}->{self.target_frame}: {exc}'
            )
            return

        self.scan_sequence += 1
        for door in self.doors:
            is_open, beyond, total = self._classify_door(scan, transform, door)
            self.observations[door.name] = (
                is_open, beyond, total, self.scan_sequence
            )

    def _classify_door(
        self,
        scan: LaserScan,
        transform: TransformStamped,
        door: ElevatorDoor,
    ) -> Tuple[bool, int, int]:
        start_map, end_map = door.segment
        start = self._target_point_to_scan(start_map, transform)
        end = self._target_point_to_scan(end_map, transform)
        start_angle = math.atan2(start[1], start[0])
        end_angle = math.atan2(end[1], end[0])
        delta = self._normalize_angle(end_angle - start_angle)
        center = self._normalize_angle(start_angle + 0.5 * delta)
        half_width = abs(delta) * 0.5

        beyond = 0
        total = 0
        angle = scan.angle_min
        for measured_range in scan.ranges:
            if abs(self._normalize_angle(angle - center)) <= half_width:
                expected_range = self._ray_segment_intersection(angle, start, end)
                if expected_range is not None:
                    total += 1
                    if (
                        math.isinf(measured_range)
                        or (
                            math.isfinite(measured_range)
                            and measured_range > expected_range + self.open_distance_margin
                        )
                    ):
                        beyond += 1
            angle += scan.angle_increment

        is_open = total > 0 and beyond / total >= self.open_ray_fraction
        return is_open, beyond, total

    def update(self) -> py_trees.common.Status:
        if not self.started:
            self.started = True
            self.phase_started_scan = self.scan_sequence
            self.node.get_logger().info(
                'Reached elevator_lobby; classifying elevator doors from LiDAR ranges'
            )
            return py_trees.common.Status.RUNNING
        if self.failed:
            return py_trees.common.Status.FAILURE
        if not self.doors:
            self.node.get_logger().error('No elevator doors found in doors.json')
            return py_trees.common.Status.FAILURE
        if self.phase == 'complete':
            return py_trees.common.Status.SUCCESS

        if self.phase == 'wait_open' and not self.goal_active:
            candidates = []
            for door in self.doors:
                observation = self.observations.get(door.name)
                if observation is None:
                    continue
                is_open, beyond, total, sequence = observation
                if sequence > self.phase_started_scan and is_open:
                    candidates.append((beyond / total, door, beyond, total))
            if candidates:
                _, self.selected_door, beyond, total = max(
                    candidates, key=lambda candidate: candidate[0]
                )
                waypoint = self.selected_door.name.removesuffix('_door')
                self.node.get_logger().info(
                    f'{self.selected_door.name} is open: {beyond}/{total} rays are '
                    f'>{self.open_distance_margin:.2f} m beyond the closed-door range; '
                    f'navigating to {waypoint}'
                )
                self._send_goal(waypoint)
            else:
                self._log_selected_or_all('open')

        elif self.phase == 'wait_close' and self.selected_door:
            observation = self.observations.get(self.selected_door.name)
            if observation is not None:
                is_open, beyond, total, sequence = observation
                if sequence > self.phase_started_scan and not is_open and total > 0:
                    self.phase = 'wait_reopen'
                    self.phase_started_scan = sequence
                    self.last_status_log_ns = 0
                    self.node.get_logger().info(
                        f'{self.selected_door.name} is closed: only {beyond}/{total} rays '
                        f'are >{self.open_distance_margin:.2f} m beyond the closed-door range; '
                        'waiting for it to reopen'
                    )
                else:
                    self._log_observation(self.selected_door.name, 'close', observation)

        elif self.phase == 'wait_reopen' and self.selected_door and not self.goal_active:
            observation = self.observations.get(self.selected_door.name)
            if observation is not None:
                is_open, beyond, total, sequence = observation
                if sequence > self.phase_started_scan and is_open:
                    self.node.get_logger().info(
                        f'{self.selected_door.name} reopened: {beyond}/{total} rays are '
                        f'>{self.open_distance_margin:.2f} m beyond the closed-door range; '
                        'returning to elevator_lobby'
                    )
                    self._send_goal('elevator_lobby')
                else:
                    self._log_observation(self.selected_door.name, 'reopen', observation)
        return py_trees.common.Status.RUNNING

    def _log_selected_or_all(self, desired_state: str) -> None:
        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns - self.last_status_log_ns < 1_000_000_000:
            return
        self.last_status_log_ns = now_ns
        summary = ', '.join(
            f'{name}={beyond}/{total}'
            for name, (_, beyond, total, _) in sorted(self.observations.items())
        )
        self.node.get_logger().info(
            f'Waiting for an elevator door to {desired_state}; beyond/total rays: {summary}'
        )

    def _log_observation(self, name: str, desired_state: str, observation) -> None:
        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns - self.last_status_log_ns < 1_000_000_000:
            return
        self.last_status_log_ns = now_ns
        is_open, beyond, total, _ = observation
        self.node.get_logger().info(
            f'Waiting for {name} to {desired_state}: classified '
            f'{"open" if is_open else "closed"}, {beyond}/{total} rays beyond range'
        )

    def _send_goal(self, waypoint_name: str) -> None:
        waypoint = self.waypoints.get(waypoint_name)
        if waypoint is None:
            self.node.get_logger().error(f'Missing waypoint {waypoint_name}')
            self.failed = True
            return
        if not self.action_client.wait_for_server(timeout_sec=0.0):
            if not self.waiting_for_server_logged:
                self.node.get_logger().warning('Waiting for NavigateThroughPoses action server')
                self.waiting_for_server_logged = True
            return
        self.waiting_for_server_logged = False
        pose = self._pose_from_waypoint(waypoint)
        pose.header.stamp = self.node.get_clock().now().to_msg()
        goal = NavigateThroughPoses.Goal()
        goal.poses = [pose]
        self.goal_active = True
        self.goal_destination = waypoint_name
        self.action_client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.goal_active = False
            self.failed = True
            self.node.get_logger().error(f'Failed to send elevator goal: {exc}')
            return
        if not self.goal_handle.accepted:
            self.goal_active = False
            self.failed = True
            self.node.get_logger().error('Elevator navigation goal was rejected')
            return
        self.goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        self.goal_active = False
        try:
            result = future.result()
        except Exception as exc:
            self.failed = True
            self.node.get_logger().error(f'Failed to get elevator navigation result: {exc}')
            return
        if result.status != 4:
            self.failed = True
            self.node.get_logger().error(
                f'Elevator navigation failed with status {result.status}'
            )
            return
        if self.goal_destination == 'elevator_lobby':
            self.phase = 'complete'
            self.node.get_logger().info('Returned to elevator_lobby')
        else:
            self.phase = 'wait_close'
            self.phase_started_scan = self.scan_sequence
            self.last_status_log_ns = 0
            self.node.get_logger().info(
                f'Reached {self.goal_destination}; waiting for a new scan that classifies '
                'the door as closed'
            )

    def cancel_active_goal(self) -> None:
        if self.goal_active and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    @staticmethod
    def _pose_from_waypoint(waypoint: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(waypoint.get('frame_id', 'map'))
        pose.pose.position.x = float(waypoint.get('x', 0.0))
        pose.pose.position.y = float(waypoint.get('y', 0.0))
        pose.pose.position.z = float(waypoint.get('z', 0.0))
        orientation = waypoint.get('orientation', {})
        pose.pose.orientation.x = float(orientation.get('x', 0.0))
        pose.pose.orientation.y = float(orientation.get('y', 0.0))
        pose.pose.orientation.z = float(orientation.get('z', 0.0))
        pose.pose.orientation.w = float(orientation.get('w', 1.0))
        return pose

    @staticmethod
    def _target_point_to_scan(
        point: Tuple[float, float], transform: TransformStamped
    ) -> Tuple[float, float]:
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        dx = point[0] - transform.transform.translation.x
        dy = point[1] - transform.transform.translation.y
        return (
            math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
        )

    @staticmethod
    def _ray_segment_intersection(
        angle: float,
        start: Tuple[float, float],
        end: Tuple[float, float],
    ) -> Optional[float]:
        ray_x = math.cos(angle)
        ray_y = math.sin(angle)
        seg_x = end[0] - start[0]
        seg_y = end[1] - start[1]
        denominator = ray_x * seg_y - ray_y * seg_x
        if abs(denominator) <= 1e-9:
            return None
        distance = (start[0] * seg_y - start[1] * seg_x) / denominator
        segment_fraction = (start[0] * ray_y - start[1] * ray_x) / denominator
        if distance < 0.0 or segment_fraction < 0.0 or segment_fraction > 1.0:
            return None
        return distance

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))
