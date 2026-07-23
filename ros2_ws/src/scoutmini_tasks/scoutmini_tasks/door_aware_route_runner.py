#!/usr/bin/env python3
"""Door-aware route runner using py_trees_ros.

The tree follows a waypoint route in segments. At configured door pre-waypoints
(e.g. door2_pre), it stops, checks LiDAR for a closed door, waits while blocked,
and then resumes with the post-waypoint once the scan no longer detects the door.
Door checks are data-driven from map_tools/maps/<map>/doors.json.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import py_trees
import py_trees_ros
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathThroughPoses, NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class DoorDetectionState:
    closed: bool = False
    door_name: str = ''
    hit_count: int = 0
    active_door_name: str = ''


@dataclass
class DoorGeometry:
    name: str
    frame_id: str = 'map'
    pre_waypoint: str = ''
    post_waypoint: str = ''
    mode: str = 'segment'
    hinge_x: float = 0.0
    hinge_y: float = 0.0
    yaw: float = 0.0
    length: float = 1.0
    hit_tolerance: float = 0.18
    min_hits: int = 3
    max_range: float = 8.0
    max_abs_scan_angle: float = math.radians(15.0)
    forward_min: float = 0.15
    forward_max: float = 1.8
    half_width: float = 0.55
    use_front_scan_fallback: bool = True
    path_crossing_tolerance: float = 0.35

    @property
    def segment(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        end_x = self.hinge_x - math.sin(self.yaw) * self.length
        end_y = self.hinge_y + math.cos(self.yaw) * self.length
        return (self.hinge_x, self.hinge_y), (end_x, end_y)


@dataclass
class RouteStep:
    kind: str
    name: str
    poses: List[PoseStamped]
    door_name: str = ''



def load_doors(node: Node, doors_file: Path) -> List[DoorGeometry]:
    try:
        with doors_file.open('r', encoding='utf-8') as stream:
            data = json.load(stream)
    except Exception as exc:
        node.get_logger().error(f'Failed to load door config {doors_file}: {exc}')
        return []

    doors: List[DoorGeometry] = []
    for item in data.get('doors', []):
        detection = item.get('detection', {})
        if not detection.get('enabled', True):
            continue
        route_gate = item.get('route_gate', {})
        closed_pose = item.get('closed_pose', {})
        leaf = item.get('leaf', {})
        try:
            doors.append(
                DoorGeometry(
                    name=str(item['name']),
                    frame_id=str(item.get('frame_id', 'map')),
                    pre_waypoint=str(route_gate.get('pre_waypoint', '')),
                    post_waypoint=str(route_gate.get('post_waypoint', '')),
                    mode=str(detection.get('mode', 'segment')),
                    hinge_x=float(closed_pose.get('x', 0.0)),
                    hinge_y=float(closed_pose.get('y', 0.0)),
                    yaw=float(closed_pose.get('yaw', 0.0)),
                    length=float(leaf.get('length', 1.0)),
                    hit_tolerance=float(detection.get('hit_tolerance', 0.18)),
                    min_hits=int(detection.get('min_hits', 3)),
                    max_range=float(detection.get('max_range', 8.0)),
                    max_abs_scan_angle=float(detection.get('max_abs_scan_angle', math.radians(15.0))),
                    forward_min=float(detection.get('forward_min', 0.15)),
                    forward_max=float(detection.get('forward_max', 1.8)),
                    half_width=float(detection.get('half_width', 0.55)),
                    use_front_scan_fallback=bool(detection.get('front_scan_fallback', True)),
                    path_crossing_tolerance=float(route_gate.get('path_crossing_tolerance', 0.35)),
                )
            )
        except (KeyError, TypeError, ValueError) as exc:
            node.get_logger().warn(f'Skipping invalid door config entry {item}: {exc}')

    node.get_logger().info(f'Loaded {len(doors)} door definition(s) from {doors_file}')
    return doors

class DoorMonitor(py_trees.behaviour.Behaviour):
    """Update shared state when LiDAR indicates the active door is closed."""

    def __init__(
        self,
        node: Node,
        state: DoorDetectionState,
        doors_file: Path,
        scan_topic: str,
        target_frame: str,
    ) -> None:
        super().__init__(name='DoorMonitor')
        self.node = node
        self.state = state
        self.doors_file = doors_file
        self.target_frame = target_frame
        self.latest_scan: Optional[LaserScan] = None
        self.doors = load_doors(node, doors_file)
        self.doors_by_name = {door.name: door for door in self.doors}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.scan_sub = node.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self._last_closed_name = ''

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def update(self) -> py_trees.common.Status:
        self._update_detection()
        return py_trees.common.Status.SUCCESS

    def _update_detection(self) -> None:
        if not self.state.active_door_name:
            self._set_state(False, '', 0)
            return

        door = self.doors_by_name.get(self.state.active_door_name)
        if door is None or self.latest_scan is None:
            self._set_state(False, '', 0)
            return

        if door.mode == 'front_scan':
            hits = self._front_scan_hits(door, self.latest_scan)
        else:
            segment_hits = self._segment_hits(door, self.latest_scan)
            front_hits = self._front_scan_hits(door, self.latest_scan) if door.use_front_scan_fallback else 0
            hits = max(segment_hits, front_hits)

        if hits >= door.min_hits:
            self._set_state(True, door.name, hits)
        else:
            self._set_state(False, '', hits)

    def _front_scan_hits(self, door: DoorGeometry, scan: LaserScan) -> int:
        hits = 0
        angle = scan.angle_min
        for scan_range in scan.ranges:
            if math.isfinite(scan_range):
                x = scan_range * math.cos(angle)
                y = scan_range * math.sin(angle)
                if (
                    door.forward_min <= x <= door.forward_max
                    and abs(y) <= door.half_width
                    and scan_range <= door.max_range
                    and abs(angle) <= door.max_abs_scan_angle
                ):
                    hits += 1
            angle += scan.angle_increment
        return hits

    def _segment_hits(self, door: DoorGeometry, scan: LaserScan) -> int:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.node.get_logger().debug(f'No transform {scan.header.frame_id}->{self.target_frame}: {exc}')
            return 0

        hits = 0
        angle = scan.angle_min
        for scan_range in scan.ranges:
            if (
                math.isfinite(scan_range)
                and scan_range <= door.max_range
                and abs(angle) <= door.max_abs_scan_angle
            ):
                point = self._scan_point_to_target(scan_range, angle, transform)
                if self._point_near_segment(point, door.segment, door.hit_tolerance):
                    hits += 1
            angle += scan.angle_increment
        return hits

    def _set_state(self, closed: bool, door_name: str, hit_count: int) -> None:
        changed = closed != self.state.closed or door_name != self.state.door_name
        self.state.closed = closed
        self.state.door_name = door_name
        self.state.hit_count = hit_count
        if changed:
            if closed:
                self.node.get_logger().warn(
                    f'Closed door detected by LiDAR: {door_name} ({hit_count} scan hit(s)); waiting at pre-waypoint'
                )
            elif self._last_closed_name:
                self.node.get_logger().info(
                    f'Door no longer detected: {self._last_closed_name}; route can resume'
                )
        self._last_closed_name = door_name if closed else ''

    @staticmethod
    def _scan_point_to_target(
        scan_range: float,
        angle: float,
        transform: TransformStamped,
    ) -> Tuple[float, float]:
        local_x = scan_range * math.cos(angle)
        local_y = scan_range * math.sin(angle)
        yaw = DoorMonitor._yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        target_x = transform.transform.translation.x + cos_yaw * local_x - sin_yaw * local_y
        target_y = transform.transform.translation.y + sin_yaw * local_x + cos_yaw * local_y
        return target_x, target_y

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _point_near_segment(
        point: Tuple[float, float],
        segment: Tuple[Tuple[float, float], Tuple[float, float]],
        tolerance: float,
    ) -> bool:
        px, py = point
        (ax, ay), (bx, by) = segment
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            return math.hypot(px - ax, py - ay) <= tolerance
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        closest_x = ax + t * dx
        closest_y = ay + t * dy
        return math.hypot(px - closest_x, py - closest_y) <= tolerance


class RouteNavigator(py_trees.behaviour.Behaviour):
    """Navigate route segments, waiting at configured door checks."""

    def __init__(
        self,
        node: Node,
        door_state: DoorDetectionState,
        route_steps: List[RouteStep],
        action_name: str,
        planner_action_name: str,
        auto_insert_door_waypoints: bool,
        start_delay_sec: float,
        wait_for_server_sec: float,
        door_settle_sec: float,
        clear_hit_fraction: float,
        clear_delay_sec: float,
        doors: List[DoorGeometry],
    ) -> None:
        super().__init__(name='RouteNavigator')
        self.node = node
        self.door_state = door_state
        self.route_steps = route_steps
        self.action_name = action_name
        self.planner_action_name = planner_action_name
        self.auto_insert_door_waypoints = auto_insert_door_waypoints
        self.start_delay_sec = start_delay_sec
        self.wait_for_server_sec = wait_for_server_sec
        self.door_settle_sec = door_settle_sec
        self.clear_hit_fraction = clear_hit_fraction
        self.clear_delay_sec = clear_delay_sec
        self.doors = doors
        self.action_client = ActionClient(node, NavigateThroughPoses, action_name)
        self.planner_client = ActionClient(node, ComputePathThroughPoses, planner_action_name)
        self.start_time = node.get_clock().now()
        self.goal_handle = None
        self.goal_active = False
        self.done = False
        self.step_index = 0
        self.active_step_index = -1
        self.waiting_started_ns = 0
        self.last_wait_log_ns = 0
        self.max_wait_hit_count = 0
        self.clear_started_ns = 0
        self.last_action_wait_log_ns = 0
        self.plan_check_active = False
        self.plan_check_complete = False
        self.plan_check_failed = False
        self.plan_check_path = None
        self.plan_check_step_index = -1
        self.plan_checked_step_index = -1
        self.last_planner_wait_log_ns = 0
        self.cleared_doors: Set[str] = set()

    def update(self) -> py_trees.common.Status:
        if self.done:
            return py_trees.common.Status.SUCCESS

        if self.step_index >= len(self.route_steps):
            self.done = True
            self.door_state.active_door_name = ''
            self.node.get_logger().info('Door-aware route complete')
            return py_trees.common.Status.SUCCESS

        step = self.route_steps[self.step_index]
        if step.kind == 'door_check':
            return self._update_door_check(step)

        if step.door_name:
            self.door_state.active_door_name = step.door_name

        if self.goal_active:
            return py_trees.common.Status.RUNNING

        elapsed_sec = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_sec < self.start_delay_sec:
            return py_trees.common.Status.RUNNING

        if not self.action_client.wait_for_server(timeout_sec=0.0):
            now_ns = self.node.get_clock().now().nanoseconds
            if now_ns - self.last_action_wait_log_ns > 1_000_000_000:
                self.last_action_wait_log_ns = now_ns
                self.node.get_logger().warn(
                    f'Waiting for {self.action_name} action server ({elapsed_sec:.1f}s elapsed)'
                )
            if elapsed_sec > self.wait_for_server_sec:
                self.node.get_logger().warn(
                    f'Waiting for {self.action_name} action server exceeded {self.wait_for_server_sec:.1f}s'
                )
            return py_trees.common.Status.RUNNING

        self.last_action_wait_log_ns = 0
        if self._check_planned_path_before_step(step):
            return py_trees.common.Status.RUNNING

        self._send_step(step)
        return py_trees.common.Status.RUNNING

    def _check_planned_path_before_step(self, step: RouteStep) -> bool:
        if not self.auto_insert_door_waypoints or step.kind != 'navigate' or step.door_name or not step.poses:
            return False
        if self.plan_checked_step_index == self.step_index:
            return False

        if self.plan_check_active:
            if not self.plan_check_complete:
                return True

            self.plan_check_active = False
            self.plan_check_step_index = -1
            if self.plan_check_failed or self.plan_check_path is None:
                self.plan_check_path = None
                door = self._door_for_failed_plan(step)
                self.plan_checked_step_index = self.step_index
                if door is not None and self._insert_door_before_step(step, door, None):
                    self.plan_checked_step_index = -1
                    return True
                return False

            crossed_door = self._door_crossed_by_planned_path(self.plan_check_path, step)
            self.plan_check_path = None
            self.plan_checked_step_index = self.step_index
            if crossed_door is None:
                return False

            door, crossing_segment = crossed_door
            if self._insert_door_before_step(step, door, crossing_segment):
                self.plan_checked_step_index = -1
                return True
            return False

        if not self.planner_client.wait_for_server(timeout_sec=0.0):
            now_ns = self.node.get_clock().now().nanoseconds
            if now_ns - self.last_planner_wait_log_ns > 1_000_000_000:
                self.last_planner_wait_log_ns = now_ns
                self.node.get_logger().warn(f'Waiting for {self.planner_action_name} planner action server')
            return True

        self.last_planner_wait_log_ns = 0
        self._send_plan_check(step)
        return True

    def _send_plan_check(self, step: RouteStep) -> None:
        now = self.node.get_clock().now().to_msg()
        for pose in step.poses:
            pose.header.stamp = now

        goal = ComputePathThroughPoses.Goal()
        goal.goals = list(step.poses)
        goal.planner_id = ''
        goal.use_start = False
        self.plan_check_active = True
        self.plan_check_complete = False
        self.plan_check_failed = False
        self.plan_check_path = None
        self.plan_check_step_index = self.step_index
        self.node.get_logger().debug(f'Checking Nav2 planned path for doors before {step.name}')
        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(self._plan_goal_response_cb)

    def _plan_goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.get_logger().warn(f'Failed to request planned path for door check: {exc}')
            self.plan_check_failed = True
            self.plan_check_complete = True
            return

        if not goal_handle.accepted:
            self.node.get_logger().warn('Planned path door check was rejected by Nav2 planner')
            self.plan_check_failed = True
            self.plan_check_complete = True
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._plan_result_cb)

    def _plan_result_cb(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.node.get_logger().warn(f'Failed to get planned path for door check: {exc}')
            self.plan_check_failed = True
            self.plan_check_complete = True
            return

        if result.status != 4:
            self.node.get_logger().warn(
                f'Planned path door check finished with status {result.status}; '
                'will try an in-memory door insertion instead of sending the blocked goal'
            )
            self.plan_check_failed = True
            self.plan_check_complete = True
            return

        self.plan_check_path = result.result.path
        self.plan_check_complete = True

    def _step_already_enters_door_exit(self, step: RouteStep, door: DoorGeometry) -> bool:
        if not step.poses:
            return False
        return step.name in (door.pre_waypoint, door.post_waypoint)

    def _pose_named_like(self, template_pose: PoseStamped, waypoint_name: str) -> Optional[PoseStamped]:
        waypoint = self.node.waypoints_by_name.get(waypoint_name)
        if waypoint is None:
            return None
        pose = self.node._pose_from_waypoint(waypoint)
        pose.header.stamp = template_pose.header.stamp
        return pose

    def _door_crossed_by_planned_path(
        self,
        path,
        step: RouteStep,
    ) -> Optional[Tuple[DoorGeometry, Tuple[Tuple[float, float], Tuple[float, float]]]]:
        poses = getattr(path, 'poses', [])
        if len(poses) < 2:
            return None

        for index in range(len(poses) - 1):
            start = poses[index].pose.position
            end = poses[index + 1].pose.position
            path_segment = ((start.x, start.y), (end.x, end.y))
            for door in self.doors:
                if not door.pre_waypoint or not door.post_waypoint:
                    continue
                if door.name in self.cleared_doors:
                    continue
                if self._step_already_enters_door_exit(step, door):
                    continue

                crossed_segment = self._crossed_door_segment(path_segment, door)
                if crossed_segment:
                    direction_segment, direction_indices = self._direction_segment_around_crossing(poses, index)
                    self.node.get_logger().info(
                        f'Nav2 planned path crosses {door.name} {crossed_segment} '
                        f'between path point {index} and {index + 1}; '
                        f'direction check uses path point {direction_indices[0]} and {direction_indices[1]}'
                    )
                    return door, direction_segment
        self.node.get_logger().info(
            f'Nav2 planned path to {step.name} with {len(poses)} point(s) did not cross a configured door'
        )
        return None

    @staticmethod
    def _direction_segment_around_crossing(
        poses,
        crossing_index: int,
    ) -> Tuple[Tuple[Tuple[float, float], Tuple[float, float]], Tuple[int, int]]:
        start_index = max(0, crossing_index - 2)
        end_index = min(len(poses) - 1, crossing_index + 3)
        start = poses[start_index].pose.position
        end = poses[end_index].pose.position
        return ((start.x, start.y), (end.x, end.y)), (start_index, end_index)

    def _crossed_door_segment(
        self,
        path_segment: Tuple[Tuple[float, float], Tuple[float, float]],
        door: DoorGeometry,
    ) -> str:
        if DoorAwareRouteRunner._segments_intersect(path_segment, door.segment):
            return 'closed leaf segment'
        if DoorAwareRouteRunner._segment_distance(path_segment, door.segment) <= door.path_crossing_tolerance:
            return f'near closed leaf segment ({door.path_crossing_tolerance:.2f} m)'

        gate_segment = self._door_gate_segment(door)
        if gate_segment and DoorAwareRouteRunner._segments_intersect(path_segment, gate_segment):
            return 'route gate segment'
        if gate_segment and DoorAwareRouteRunner._segment_distance(path_segment, gate_segment) <= door.path_crossing_tolerance:
            return f'near route gate segment ({door.path_crossing_tolerance:.2f} m)'
        return ''

    def _door_gate_segment(self, door: DoorGeometry) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        pre_waypoint = self.node.waypoints_by_name.get(door.pre_waypoint)
        post_waypoint = self.node.waypoints_by_name.get(door.post_waypoint)
        if pre_waypoint is None or post_waypoint is None:
            return None
        return (
            (float(pre_waypoint.get('x', 0.0)), float(pre_waypoint.get('y', 0.0))),
            (float(post_waypoint.get('x', 0.0)), float(post_waypoint.get('y', 0.0))),
        )

    def _door_for_failed_plan(self, step: RouteStep) -> Optional[DoorGeometry]:
        candidates = [
            door
            for door in self.doors
            if door.pre_waypoint
            and door.post_waypoint
            and door.name not in self.cleared_doors
            and not self._step_already_enters_door_exit(step, door)
        ]
        if not candidates:
            self.node.get_logger().warn(
                f'Planner could not generate a path to {step.name}, and no uncleared door candidate is available'
            )
            return None

        if len(candidates) == 1:
            door = candidates[0]
            self.node.get_logger().warn(
                f'Planner could not generate a path to {step.name}; trying configured door {door.name}'
            )
            return door

        target = step.poses[-1].pose.position
        def distance_to_target(door: DoorGeometry) -> float:
            gate_segment = self._door_gate_segment(door)
            if gate_segment is not None:
                return DoorAwareRouteRunner._point_segment_distance((target.x, target.y), gate_segment)
            return DoorAwareRouteRunner._point_segment_distance((target.x, target.y), door.segment)

        door = min(candidates, key=distance_to_target)
        self.node.get_logger().warn(
            f'Planner could not generate a path to {step.name}; trying nearest configured door {door.name}'
        )
        return door

    def _door_steps_for_step(
        self,
        step: RouteStep,
        door: DoorGeometry,
        crossing_segment: Optional[Tuple[Tuple[float, float], Tuple[float, float]]],
    ) -> List[RouteStep]:
        pre_pose = self._pose_named_like(step.poses[0], door.pre_waypoint)
        post_pose = self._pose_named_like(step.poses[0], door.post_waypoint)
        if pre_pose is None or post_pose is None:
            self.node.get_logger().warn(
                f'Planned path crosses {door.name}, but pre/post poses are unavailable; continuing original step'
            )
            return []

        first_name = door.pre_waypoint
        second_name = door.post_waypoint
        first_pose = pre_pose
        second_pose = post_pose
        reverse = self._should_enter_from_post_side(step, door, crossing_segment)
        if reverse:
            first_name = door.post_waypoint
            second_name = door.pre_waypoint
            first_pose = self._pose_with_yaw_offset(post_pose, math.pi)
            second_pose = self._pose_with_yaw_offset(pre_pose, math.pi)

        return [
            RouteStep(kind='navigate', name=first_name, poses=[first_pose], door_name=door.name),
            RouteStep(kind='door_check', name=first_name, poses=[], door_name=door.name),
            RouteStep(kind='navigate', name=second_name, poses=[second_pose]),
            step,
        ]

    def _should_enter_from_post_side(
        self,
        step: RouteStep,
        door: DoorGeometry,
        crossing_segment: Optional[Tuple[Tuple[float, float], Tuple[float, float]]],
    ) -> bool:
        pre_waypoint = self.node.waypoints_by_name.get(door.pre_waypoint)
        post_waypoint = self.node.waypoints_by_name.get(door.post_waypoint)
        if pre_waypoint is None or post_waypoint is None:
            return False
        pre_xy = (float(pre_waypoint.get('x', 0.0)), float(pre_waypoint.get('y', 0.0)))
        post_xy = (float(post_waypoint.get('x', 0.0)), float(post_waypoint.get('y', 0.0)))
        if crossing_segment is not None:
            start_xy, end_xy = crossing_segment
        else:
            start_xy = self._route_start_xy_for_step(step)
            end = step.poses[-1].pose.position
            end_xy = (end.x, end.y)

        path_vector = (end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
        door_vector = (post_xy[0] - pre_xy[0], post_xy[1] - pre_xy[1])
        dot = path_vector[0] * door_vector[0] + path_vector[1] * door_vector[1]
        reverse = dot < 0.0
        self.node.get_logger().info(
            f'Door {door.name} direction check: '
            f'path_start=({start_xy[0]:.2f}, {start_xy[1]:.2f}), '
            f'path_end=({end_xy[0]:.2f}, {end_xy[1]:.2f}), '
            f'pre=({pre_xy[0]:.2f}, {pre_xy[1]:.2f}), '
            f'post=({post_xy[0]:.2f}, {post_xy[1]:.2f}), '
            f'path_vector=({path_vector[0]:.2f}, {path_vector[1]:.2f}), '
            f'door_vector=({door_vector[0]:.2f}, {door_vector[1]:.2f}), '
            f'dot={dot:.3f}; using '
            f'{door.post_waypoint if reverse else door.pre_waypoint} first'
        )
        return reverse

    def _route_start_xy_for_step(self, step: RouteStep) -> Tuple[float, float]:
        for previous_index in range(self.step_index - 1, -1, -1):
            previous_step = self.route_steps[previous_index]
            if previous_step.poses:
                point = previous_step.poses[-1].pose.position
                return point.x, point.y
        point = step.poses[0].pose.position
        return point.x, point.y

    @staticmethod
    def _pose_with_yaw_offset(pose: PoseStamped, yaw_offset: float) -> PoseStamped:
        adjusted = PoseStamped()
        adjusted.header = pose.header
        adjusted.pose.position.x = pose.pose.position.x
        adjusted.pose.position.y = pose.pose.position.y
        adjusted.pose.position.z = pose.pose.position.z
        q = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ) + yaw_offset
        adjusted.pose.orientation.x = 0.0
        adjusted.pose.orientation.y = 0.0
        adjusted.pose.orientation.z = math.sin(yaw / 2.0)
        adjusted.pose.orientation.w = math.cos(yaw / 2.0)
        return adjusted

    def _insert_door_before_step(
        self,
        step: RouteStep,
        door: DoorGeometry,
        crossing_segment: Optional[Tuple[Tuple[float, float], Tuple[float, float]]],
    ) -> bool:
        inserted_steps = self._door_steps_for_step(step, door, crossing_segment)
        if not inserted_steps:
            return False
        self.route_steps[self.step_index:self.step_index + 1] = inserted_steps
        first = inserted_steps[0].name
        second = inserted_steps[2].name
        self.node.get_logger().warn(
            f'Inserted {first}->{second} before {step.name} '
            f'from Nav2 planned path crossing of {door.name}'
        )
        return True

    def _update_door_check(self, step: RouteStep) -> py_trees.common.Status:
        self.door_state.active_door_name = step.door_name
        now_ns = self.node.get_clock().now().nanoseconds
        if self.waiting_started_ns == 0:
            self.waiting_started_ns = now_ns
            self.last_wait_log_ns = 0
            self.max_wait_hit_count = self.door_state.hit_count
            self.node.get_logger().info(
                f'Stopped at {step.name}; checking LiDAR for door {step.door_name}'
            )
            return py_trees.common.Status.RUNNING

        # Give the scan monitor a moment to tick at the stopped pose before deciding clear.
        if (now_ns - self.waiting_started_ns) / 1e9 < self.door_settle_sec:
            return py_trees.common.Status.RUNNING

        self.max_wait_hit_count = max(self.max_wait_hit_count, self.door_state.hit_count)
        clear_threshold = self.max_wait_hit_count * self.clear_hit_fraction
        if self.max_wait_hit_count > 0 and self.door_state.hit_count > clear_threshold:
            self.clear_started_ns = 0
            if now_ns - self.last_wait_log_ns > 1_000_000_000:
                self.last_wait_log_ns = now_ns
                self.node.get_logger().info(
                    f'Waiting for {step.door_name} to clear '
                    f'({self.door_state.hit_count}/{self.max_wait_hit_count} scan hit(s), '
                    f'clear at <= {clear_threshold:.1f})'
                )
            return py_trees.common.Status.RUNNING

        if self.clear_started_ns == 0:
            self.clear_started_ns = now_ns
            self.node.get_logger().info(
                f'Door {step.door_name} is below clear threshold '
                f'({self.door_state.hit_count}/{self.max_wait_hit_count} scan hit(s)); '
                f'waiting {self.clear_delay_sec:.1f}s before continuing'
            )
            return py_trees.common.Status.RUNNING

        clear_elapsed_sec = (now_ns - self.clear_started_ns) / 1e9
        if clear_elapsed_sec < self.clear_delay_sec:
            return py_trees.common.Status.RUNNING

        self.node.get_logger().info(f'Door {step.door_name} is clear; continuing through doorway')
        self.cleared_doors.add(step.door_name)
        self.door_state.active_door_name = ''
        self.waiting_started_ns = 0
        self.last_wait_log_ns = 0
        self.max_wait_hit_count = 0
        self.clear_started_ns = 0
        self.step_index += 1
        return py_trees.common.Status.RUNNING

    def _send_step(self, step: RouteStep) -> None:
        if not step.poses:
            self.step_index += 1
            return

        now = self.node.get_clock().now().to_msg()
        for pose in step.poses:
            pose.header.stamp = now

        goal = NavigateThroughPoses.Goal()
        goal.poses = list(step.poses)
        self.active_step_index = self.step_index
        self.goal_active = True
        if step.door_name:
            self.door_state.active_door_name = step.door_name
        final_pose = step.poses[-1].pose.position
        self.node.get_logger().info(
            f'Sending route segment {step.name} with {len(step.poses)} pose(s); '
            f'final target=({final_pose.x:.2f}, {final_pose.y:.2f}); door={step.door_name or "none"}'
        )
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.goal_active = False
            self.node.get_logger().error(f'Failed to send route segment: {exc}')
            return

        if not self.goal_handle.accepted:
            self.goal_active = False
            self.node.get_logger().error('Door-aware route segment was rejected by Nav2')
            return

        self.node.get_logger().info('Door-aware route segment accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        self.goal_active = False
        try:
            result = future.result()
        except Exception as exc:
            self.node.get_logger().error(f'Failed to get route segment result: {exc}')
            return

        self.node.get_logger().info(f'Route segment finished with Nav2 status code {result.status}')
        if self.active_step_index == self.step_index:
            self.step_index += 1
        self.active_step_index = -1


class DoorAwareRouteRunner(Node):
    """ROS node hosting a py_trees_ros behavior tree for door-aware route navigation."""

    def __init__(self) -> None:
        super().__init__('door_aware_route_runner')
        self.declare_parameter('route_name', 'route1')
        self.declare_parameter('map_name', 'fuse_3rd')
        self.declare_parameter('doors_file', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('action_name', '/navigate_through_poses')
        self.declare_parameter('planner_action_name', '/compute_path_through_poses')
        self.declare_parameter('start_delay_sec', 12.0)
        self.declare_parameter('wait_for_server_sec', 60.0)
        self.declare_parameter('door_settle_sec', 0.6)
        self.declare_parameter('door_clear_hit_fraction', 3.0 / 5.0)
        self.declare_parameter('door_clear_delay_sec', 1.0)
        self.declare_parameter('auto_insert_door_waypoints', True)
        self.declare_parameter('tick_period_sec', 0.2)

        self.map_name = str(self.get_parameter('map_name').value)
        self.route_name = str(self.get_parameter('route_name').value)
        doors_file_param = str(self.get_parameter('doors_file').value)
        self.doors_file = Path(doors_file_param) if doors_file_param else self._default_doors_file()
        self.door_state = DoorDetectionState()
        self.doors = load_doors(self, self.doors_file)
        self.waypoints_by_name: Dict[str, dict] = {}
        self.route_steps = self._load_route_steps()
        self.get_logger().info(
            f'Door-aware runner ready: route={self.route_name}, map={self.map_name}, '
            f'target_frame={self.get_parameter("target_frame").value}, '
            f'base_frame={self.get_parameter("base_frame").value}, action={self.get_parameter("action_name").value}'
        )

        root = self._make_tree_root()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        self._setup_tree()
        self.tick_timer = self.create_timer(float(self.get_parameter('tick_period_sec').value), self._tick_tree)

    def _default_doors_file(self) -> Path:
        map_tools_share = Path(get_package_share_directory('map_tools'))
        return map_tools_share / 'maps' / self.map_name / 'doors.json'

    def _param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _load_route_steps(self) -> List[RouteStep]:
        map_tools_share = Path(get_package_share_directory('map_tools'))
        route_file = map_tools_share / 'maps' / self.map_name / 'routes' / f'{self.route_name}.yaml'
        waypoint_file = map_tools_share / 'maps' / self.map_name / f'{self.map_name}_waypoints.json'

        with route_file.open('r', encoding='utf-8') as stream:
            route_data = yaml.safe_load(stream) or {}
        route_names = route_data if isinstance(route_data, list) else route_data.get('waypoints', [])
        route_names = [item if isinstance(item, str) else item.get('name', '') for item in route_names]
        route_names = [name.strip() for name in route_names if isinstance(name, str) and name.strip()]

        with waypoint_file.open('r', encoding='utf-8') as stream:
            waypoint_data = json.load(stream)
        waypoints = {str(item.get('name', '')).strip(): item for item in waypoint_data}
        self.waypoints_by_name = waypoints

        door_by_pre = {door.pre_waypoint: door for door in self.doors if door.pre_waypoint}
        steps: List[RouteStep] = []
        missing: List[str] = []

        for name in route_names:
            waypoint = waypoints.get(name)
            if waypoint is None:
                missing.append(name)
                continue

            door = door_by_pre.get(name)
            steps.append(RouteStep(
                kind='navigate',
                name=name,
                poses=[self._pose_from_waypoint(waypoint)],
                door_name=door.name if door is not None else '',
            ))
            if door is not None:
                steps.append(RouteStep(kind='door_check', name=name, poses=[], door_name=door.name))

        if missing:
            raise RuntimeError(f'Missing waypoint(s) for route {self.route_name}: {missing}')
        self.get_logger().info(
            f'Loaded route {self.route_name} as {len(steps)} behavior step(s): {[step.name for step in steps]}'
        )
        return steps

    @classmethod
    def _segment_distance(
        cls,
        first: Tuple[Tuple[float, float], Tuple[float, float]],
        second: Tuple[Tuple[float, float], Tuple[float, float]],
    ) -> float:
        if cls._segments_intersect(first, second):
            return 0.0
        return min(
            cls._point_segment_distance(first[0], second),
            cls._point_segment_distance(first[1], second),
            cls._point_segment_distance(second[0], first),
            cls._point_segment_distance(second[1], first),
        )

    @staticmethod
    def _point_segment_distance(
        point: Tuple[float, float],
        segment: Tuple[Tuple[float, float], Tuple[float, float]],
    ) -> float:
        px, py = point
        (ax, ay), (bx, by) = segment
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        closest_x = ax + t * dx
        closest_y = ay + t * dy
        return math.hypot(px - closest_x, py - closest_y)

    @classmethod
    def _segments_intersect(
        cls,
        first: Tuple[Tuple[float, float], Tuple[float, float]],
        second: Tuple[Tuple[float, float], Tuple[float, float]],
    ) -> bool:
        a, b = first
        c, d = second
        o1 = cls._orientation(a, b, c)
        o2 = cls._orientation(a, b, d)
        o3 = cls._orientation(c, d, a)
        o4 = cls._orientation(c, d, b)

        if o1 * o2 < 0.0 and o3 * o4 < 0.0:
            return True
        return (
            abs(o1) <= 1e-9 and cls._on_segment(a, c, b)
            or abs(o2) <= 1e-9 and cls._on_segment(a, d, b)
            or abs(o3) <= 1e-9 and cls._on_segment(c, a, d)
            or abs(o4) <= 1e-9 and cls._on_segment(c, b, d)
        )

    @staticmethod
    def _orientation(
        a: Tuple[float, float],
        b: Tuple[float, float],
        c: Tuple[float, float],
    ) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    @staticmethod
    def _on_segment(
        a: Tuple[float, float],
        b: Tuple[float, float],
        c: Tuple[float, float],
    ) -> bool:
        return (
            min(a[0], c[0]) - 1e-9 <= b[0] <= max(a[0], c[0]) + 1e-9
            and min(a[1], c[1]) - 1e-9 <= b[1] <= max(a[1], c[1]) + 1e-9
        )

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

    def _make_tree_root(self):
        try:
            root = py_trees.composites.Sequence(name='DoorAwareRoute', memory=False)
        except TypeError:
            root = py_trees.composites.Sequence(name='DoorAwareRoute')
        root.add_children([
            DoorMonitor(
                node=self,
                state=self.door_state,
                doors_file=self.doors_file,
                scan_topic=str(self.get_parameter('scan_topic').value),
                target_frame=str(self.get_parameter('target_frame').value),
            ),
            RouteNavigator(
                node=self,
                door_state=self.door_state,
                route_steps=self.route_steps,
                action_name=str(self.get_parameter('action_name').value),
                planner_action_name=str(self.get_parameter('planner_action_name').value),
                auto_insert_door_waypoints=self._param_bool('auto_insert_door_waypoints'),
                start_delay_sec=float(self.get_parameter('start_delay_sec').value),
                wait_for_server_sec=float(self.get_parameter('wait_for_server_sec').value),
                door_settle_sec=float(self.get_parameter('door_settle_sec').value),
                clear_hit_fraction=float(self.get_parameter('door_clear_hit_fraction').value),
                clear_delay_sec=float(self.get_parameter('door_clear_delay_sec').value),
                doors=self.doors,
            ),
        ])
        return root

    def _setup_tree(self) -> None:
        try:
            self.tree.setup(node=self, timeout=15.0)
        except TypeError:
            self.tree.setup(timeout=15.0)

    def _tick_tree(self) -> None:
        self.tree.tick()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoorAwareRouteRunner()
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
