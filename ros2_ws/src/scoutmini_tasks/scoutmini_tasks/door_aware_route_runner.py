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
from typing import Dict, List, Optional, Tuple

import py_trees
import py_trees_ros
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateThroughPoses
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
    max_abs_scan_angle: float = math.pi
    forward_min: float = 0.15
    forward_max: float = 1.8
    half_width: float = 0.55

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
                    max_abs_scan_angle=float(detection.get('max_abs_scan_angle', math.pi)),
                    forward_min=float(detection.get('forward_min', 0.15)),
                    forward_max=float(detection.get('forward_max', 1.8)),
                    half_width=float(detection.get('half_width', 0.55)),
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
            hits = self._segment_hits(door, self.latest_scan)

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
        start_delay_sec: float,
        wait_for_server_sec: float,
        door_settle_sec: float,
        clear_hit_fraction: float,
    ) -> None:
        super().__init__(name='RouteNavigator')
        self.node = node
        self.door_state = door_state
        self.route_steps = route_steps
        self.action_name = action_name
        self.start_delay_sec = start_delay_sec
        self.wait_for_server_sec = wait_for_server_sec
        self.door_settle_sec = door_settle_sec
        self.clear_hit_fraction = clear_hit_fraction
        self.action_client = ActionClient(node, NavigateThroughPoses, action_name)
        self.start_time = node.get_clock().now()
        self.goal_handle = None
        self.goal_active = False
        self.done = False
        self.step_index = 0
        self.active_step_index = -1
        self.waiting_started_ns = 0
        self.last_wait_log_ns = 0
        self.max_wait_hit_count = 0

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

        if self.goal_active:
            return py_trees.common.Status.RUNNING

        elapsed_sec = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_sec < self.start_delay_sec:
            return py_trees.common.Status.RUNNING

        if not self.action_client.wait_for_server(timeout_sec=0.0):
            if elapsed_sec > self.wait_for_server_sec:
                self.node.get_logger().warn(
                    f'Waiting for {self.action_name} action server exceeded {self.wait_for_server_sec:.1f}s'
                )
            return py_trees.common.Status.RUNNING

        self._send_step(step)
        return py_trees.common.Status.RUNNING

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
            if now_ns - self.last_wait_log_ns > 1_000_000_000:
                self.last_wait_log_ns = now_ns
                self.node.get_logger().info(
                    f'Waiting for {step.door_name} to clear '
                    f'({self.door_state.hit_count}/{self.max_wait_hit_count} scan hit(s), '
                    f'clear at <= {clear_threshold:.1f})'
                )
            return py_trees.common.Status.RUNNING

        self.node.get_logger().info(f'Door {step.door_name} is clear; continuing to post waypoint')
        self.door_state.active_door_name = ''
        self.waiting_started_ns = 0
        self.last_wait_log_ns = 0
        self.max_wait_hit_count = 0
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
        self.node.get_logger().info(
            f'Sending route segment {step.name} with {len(step.poses)} pose(s)'
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
        self.declare_parameter('action_name', '/navigate_through_poses')
        self.declare_parameter('start_delay_sec', 12.0)
        self.declare_parameter('wait_for_server_sec', 60.0)
        self.declare_parameter('door_settle_sec', 0.6)
        self.declare_parameter('door_clear_hit_fraction', 1.0 / 3.0)
        self.declare_parameter('tick_period_sec', 0.2)

        self.map_name = str(self.get_parameter('map_name').value)
        self.route_name = str(self.get_parameter('route_name').value)
        doors_file_param = str(self.get_parameter('doors_file').value)
        self.doors_file = Path(doors_file_param) if doors_file_param else self._default_doors_file()
        self.door_state = DoorDetectionState()
        self.doors = load_doors(self, self.doors_file)
        self.route_steps = self._load_route_steps()

        root = self._make_tree_root()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        self._setup_tree()
        self.tick_timer = self.create_timer(float(self.get_parameter('tick_period_sec').value), self._tick_tree)

    def _default_doors_file(self) -> Path:
        map_tools_share = Path(get_package_share_directory('map_tools'))
        return map_tools_share / 'maps' / self.map_name / 'doors.json'

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

        door_by_pre = {door.pre_waypoint: door for door in self.doors if door.pre_waypoint}
        door_posts = {door.post_waypoint for door in self.doors if door.post_waypoint}
        steps: List[RouteStep] = []
        current_poses: List[PoseStamped] = []
        current_names: List[str] = []
        missing: List[str] = []

        for name in route_names:
            waypoint = waypoints.get(name)
            if waypoint is None:
                missing.append(name)
                continue

            if name in door_posts and not current_poses:
                # The corresponding pre-waypoint segment already ended before a door check.
                current_names.append(name)
                current_poses.append(self._pose_from_waypoint(waypoint))
                continue

            current_names.append(name)
            current_poses.append(self._pose_from_waypoint(waypoint))

            door = door_by_pre.get(name)
            if door is not None:
                steps.append(RouteStep(kind='navigate', name=' -> '.join(current_names), poses=current_poses))
                steps.append(RouteStep(kind='door_check', name=name, poses=[], door_name=door.name))
                current_poses = []
                current_names = []

        if current_poses:
            steps.append(RouteStep(kind='navigate', name=' -> '.join(current_names), poses=current_poses))

        if missing:
            raise RuntimeError(f'Missing waypoint(s) for route {self.route_name}: {missing}')
        self.get_logger().info(
            f'Loaded route {self.route_name} as {len(steps)} behavior step(s): {[step.name for step in steps]}'
        )
        return steps

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
                start_delay_sec=float(self.get_parameter('start_delay_sec').value),
                wait_for_server_sec=float(self.get_parameter('wait_for_server_sec').value),
                door_settle_sec=float(self.get_parameter('door_settle_sec').value),
                clear_hit_fraction=float(self.get_parameter('door_clear_hit_fraction').value),
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
