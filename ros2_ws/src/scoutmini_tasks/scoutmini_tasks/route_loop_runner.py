#!/usr/bin/env python3
"""Follow a named waypoint route using Nav2.

The route is defined in YAML as a list of waypoint names. The runner subscribes
to /map_name topic to determine the current map and constructs the route YAML
path accordingly, then queries the waypoint_server service (from map_tools) to
resolve waypoint names to coordinates. If the service is unavailable or stalls,
the runner can fall back to the installed waypoint JSON for the active map.
"""

import threading
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from map_interfaces.srv import GetWaypointsByName
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import yaml


class RouteLoopRunner(Node):
    """Load a named route and navigate it using Nav2.

    This node:
    1. Subscribes to /map_name to determine the current map
    2. Loads the YAML route file from map_tools/maps/<map_name>/routes/<route_name>.yaml
    3. Resolves waypoint names to PoseStamped objects through a service or file fallback
    4. Sends NavigateThroughPoses goals to Nav2 and optionally repeats the route
    """

    def __init__(self) -> None:
        """Initialize the route runner and set up subscriptions/clients."""
        super().__init__('route_loop_runner')

        self.declare_parameter('route_name', 'route')
        self.declare_parameter('map_name', 'fuse_3rd')
        self.declare_parameter('action_name', '/navigate_through_poses')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('loop', False)
        self.declare_parameter('repeat_delay_sec', 1.0)
        self.declare_parameter('start_delay_sec', 0.0)
        self.declare_parameter('wait_for_server_sec', 30.0)
        self.declare_parameter('skip_missing_waypoints', False)

        self.route_name = (
            self.get_parameter('route_name').get_parameter_value().string_value.strip()
        )
        self.initial_map_name = (
            self.get_parameter('map_name').get_parameter_value().string_value.strip()
        )
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.repeat_delay_sec = (
            self.get_parameter('repeat_delay_sec').get_parameter_value().double_value
        )
        self.start_delay_sec = (
            self.get_parameter('start_delay_sec').get_parameter_value().double_value
        )
        self.wait_for_server_sec = (
            self.get_parameter('wait_for_server_sec').get_parameter_value().double_value
        )
        self.skip_missing_waypoints = (
            self.get_parameter('skip_missing_waypoints').get_parameter_value().bool_value
        )

        self.current_map_name = ''
        self.stop_event = threading.Event()
        self._route_waypoint_names: List[str] = []
        self._route_poses: List[PoseStamped] = []
        self._route_map_name = ''
        self._resolving_waypoints = False
        self._last_resolve_attempt_ns = 0
        self._resolve_request_start_ns = 0
        self._resolve_retry_period_sec = 1.0
        self._resolve_timeout_sec = 3.0

        self.map_name_sub = self.create_subscription(
            String,
            '/map_name',
            self._map_name_sub_cb,
            qos_profile=QoSProfile(
                depth=10,
                reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._action_client = ActionClient(self, NavigateThroughPoses, self.action_name)
        self._waypoint_client = self.create_client(GetWaypointsByName, 'get_waypoints')

        self._goal_active = False
        self._goal_sent_once = False
        self._goal_finished = False
        self._start_time = self.get_clock().now()
        self._start_timer = self.create_timer(0.5, self._tick)
        self._restart_timer: Optional[object] = None

        self.get_logger().info(f'Route runner initialized for route: {self.route_name}')
        if self.initial_map_name:
            self.current_map_name = self.initial_map_name
            self.get_logger().info(
                f'Using initial map from parameter: {self.initial_map_name}; loading route'
            )
            self._load_route_for_map(self.initial_map_name)

    def _map_name_sub_cb(self, msg: String) -> None:
        """Handle incoming /map_name topic and reload route for the new map."""
        new_map_name = msg.data.strip()
        if not new_map_name or new_map_name == self.current_map_name:
            return

        if self._goal_active or self._goal_sent_once:
            self.get_logger().warn(
                f'Map changed to: {new_map_name}, but goal was already sent; ignoring map change'
            )
            return

        self.current_map_name = new_map_name
        self.get_logger().info(f'Map changed to: {new_map_name}; reloading route')
        self._load_route_for_map(new_map_name)

    def _get_route_file_path(self, map_name: str) -> Path:
        """Construct the path to the route YAML file."""
        package_share = Path(get_package_share_directory('map_tools'))
        return package_share / 'maps' / map_name / 'routes' / f'{self.route_name}.yaml'

    def _load_route_for_map(self, map_name: str) -> None:
        """Load and parse the route YAML file, then resolve waypoint names."""
        self._route_map_name = map_name
        self._route_poses = []
        self._resolving_waypoints = False
        route_path = self._get_route_file_path(map_name)

        if not route_path.exists():
            self.get_logger().error(f'Route file not found: {route_path}')
            self._route_waypoint_names = []
            return

        try:
            with route_path.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}

            if isinstance(data, list):
                data = {'waypoints': data}

            if not isinstance(data, dict):
                raise ValueError(f'Route file must contain a YAML mapping or list: {route_path}')

            names = data.get('waypoints', [])
            if not isinstance(names, list):
                raise ValueError('Route config key "waypoints" must be a list')

            route_names: List[str] = []
            for entry in names:
                if isinstance(entry, str) and entry.strip():
                    route_names.append(entry.strip())
                elif isinstance(entry, dict) and isinstance(entry.get('name'), str):
                    if entry['name'].strip():
                        route_names.append(entry['name'].strip())

            self.get_logger().info(f'{route_names=}')
            self._route_waypoint_names = route_names
            self._resolve_route_poses()
            self.get_logger().info(
                f'Loaded route {route_path} with {len(self._route_waypoint_names)} waypoint(s)'
            )

        except Exception as exc:
            self.get_logger().error(f'Failed to load route from {route_path}: {exc}')
            self._route_waypoint_names = []
            self._route_poses = []

    def _resolve_route_poses(self) -> None:
        """Query the waypoint service to resolve waypoint names to poses."""
        if self._resolving_waypoints:
            return

        if not self._route_waypoint_names:
            self._route_poses = []
            return

        if not self._waypoint_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn('Waypoint service not available; using waypoint file fallback')
            self._resolve_route_poses_from_file()
            return

        try:
            request = GetWaypointsByName.Request()
            request.waypoint_names = self._route_waypoint_names
            self.get_logger().info(
                f'Querying waypoint service to resolve {len(request.waypoint_names)} waypoint(s)'
            )
            self._resolving_waypoints = True
            self._resolve_request_start_ns = self.get_clock().now().nanoseconds
            future = self._waypoint_client.call_async(request)
            self.get_logger().info('Waypoint service call sent; waiting for response...')
            future.add_done_callback(self._waypoint_service_cb)

        except Exception as exc:
            self._resolving_waypoints = False
            self.get_logger().error(f'Failed to call waypoint service: {exc}')
            self._route_poses = []

    def _waypoint_service_cb(self, future) -> None:
        """Handle response from waypoint service."""
        self._resolving_waypoints = False
        self.get_logger().info('Waypoint service response received; processing results...')
        try:
            response = future.result()
            self._route_poses = list(response.poses)

            if len(self._route_poses) < len(self._route_waypoint_names):
                missing = set(self._route_waypoint_names) - set(response.found_names)
                msg = f'Could not resolve {len(missing)} waypoint(s): {missing}'
                if self.skip_missing_waypoints:
                    self.get_logger().warn(msg)
                else:
                    self.get_logger().error(msg)
                    self._route_poses = []
            self.get_logger().info(
                f'Waypoint service resolved {len(self._route_poses)}/{len(self._route_waypoint_names)} waypoint(s) to poses'
            )

        except Exception as exc:
            self.get_logger().error(f'Waypoint service call failed: {exc}')
            self._route_poses = []

        self.get_logger().info(
            f'Resolved {len(self._route_poses)}/{len(self._route_waypoint_names)} waypoints to poses'
        )

    def _resolve_route_poses_from_file(self) -> None:
        """Resolve route waypoint names directly from the installed map waypoint JSON."""
        if not self._route_map_name:
            self.get_logger().error('No map name available for waypoint file fallback')
            self._route_poses = []
            return

        waypoint_path = (
            Path(get_package_share_directory('map_tools'))
            / 'maps'
            / self._route_map_name
            / f'{self._route_map_name}_waypoints.json'
        )

        try:
            with waypoint_path.open('r', encoding='utf-8') as stream:
                waypoints = yaml.safe_load(stream) or []

            by_name: Dict[str, dict] = {
                str(item.get('name', '')).strip(): item
                for item in waypoints
                if isinstance(item, dict) and str(item.get('name', '')).strip()
            }

            poses: List[PoseStamped] = []
            missing: List[str] = []
            for name in self._route_waypoint_names:
                item = by_name.get(name)
                if item is None:
                    missing.append(name)
                    continue

                pose = PoseStamped()
                pose.header.frame_id = str(item.get('frame_id', 'map'))
                pose.pose.position.x = float(item.get('x', 0.0))
                pose.pose.position.y = float(item.get('y', 0.0))
                pose.pose.position.z = float(item.get('z', 0.0))

                orientation = item.get('orientation', {})
                pose.pose.orientation.x = float(orientation.get('x', 0.0))
                pose.pose.orientation.y = float(orientation.get('y', 0.0))
                pose.pose.orientation.z = float(orientation.get('z', 0.0))
                pose.pose.orientation.w = float(orientation.get('w', 1.0))
                poses.append(pose)

            if missing and not self.skip_missing_waypoints:
                self.get_logger().error(f'Could not resolve waypoint(s) from file: {missing}')
                self._route_poses = []
                return

            if missing:
                self.get_logger().warn(f'Skipping missing waypoint(s) from file: {missing}')

            self._route_poses = poses
            self.get_logger().info(
                f'Waypoint file resolved {len(self._route_poses)}/{len(self._route_waypoint_names)} waypoint(s)'
            )

        except Exception as exc:
            self.get_logger().error(f'Failed waypoint file fallback from {waypoint_path}: {exc}')
            self._route_poses = []

    def _tick(self) -> None:
        """Main tick; sends a route goal when route and action server are ready."""
        if not self.auto_start or self._goal_finished:
            return

        if self._route_waypoint_names and not self._route_poses:
            now_ns = self.get_clock().now().nanoseconds
            if self._resolving_waypoints:
                timeout_ns = int(self._resolve_timeout_sec * 1e9)
                if now_ns - self._resolve_request_start_ns >= timeout_ns:
                    self.get_logger().warn(
                        'Waypoint service response timed out; using waypoint file fallback'
                    )
                    self._resolving_waypoints = False
                    self._resolve_route_poses_from_file()
                return

            retry_period_ns = int(self._resolve_retry_period_sec * 1e9)
            if now_ns - self._last_resolve_attempt_ns >= retry_period_ns:
                self._last_resolve_attempt_ns = now_ns
                self._resolve_route_poses()
            return

        if not self._route_poses:
            return

        if self._goal_active or self._goal_sent_once:
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            elapsed_sec = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
            if elapsed_sec > self.wait_for_server_sec:
                self.get_logger().warn(
                    f'Waiting for {self.action_name} action server exceeded {self.wait_for_server_sec:.1f}s'
                )
            return

        elapsed_sec = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed_sec < self.start_delay_sec:
            return

        self._send_route_goal()

    def _send_route_goal(self) -> None:
        """Send the route goal to the Nav2 navigate_through_poses action."""
        if not self._route_poses:
            self.get_logger().error('No route poses available to send')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = list(self._route_poses)

        self.get_logger().info(
            f'Sending route with {len(goal.poses)} pose(s) to {self.action_name}; loop={self.loop}'
        )
        self._goal_active = True
        self._goal_sent_once = True
        send_future = self._action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        """Log NavigateThroughPoses feedback as progress updates."""
        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, 'distance_remaining', None)
        poses_remaining = getattr(feedback, 'number_of_poses_remaining', None)
        recoveries = getattr(feedback, 'number_of_recoveries', None)

        parts = ['Route progress']
        if distance_remaining is not None:
            parts.append(f'distance_remaining={distance_remaining:.2f}m')
        if poses_remaining is not None:
            parts.append(f'poses_remaining={poses_remaining}')
        if recoveries is not None:
            parts.append(f'recoveries={recoveries}')

        self.get_logger().info(', '.join(parts))

    def _goal_response_cb(self, future) -> None:
        """Handle response from goal submission."""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_active = False
            self.get_logger().error(f'Failed to send route goal: {exc}')
            self._schedule_restart()
            return

        if not goal_handle.accepted:
            self._goal_active = False
            self.get_logger().error('Route goal was rejected by Nav2')
            self._schedule_restart()
            return

        self.get_logger().info('Route goal accepted; monitoring progress...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        """Handle route goal result from Nav2."""
        self._goal_active = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to retrieve route result: {exc}')
            self._schedule_restart()
            return

        status = result.status
        self.get_logger().info(f'Route finished with Nav2 status code {status}')
        self._schedule_restart()

    def _schedule_restart(self) -> None:
        """Finish or schedule another route goal when loop mode is enabled."""
        if not self.loop or self.stop_event.is_set():
            self._goal_finished = True
            return

        self._goal_sent_once = False
        self._goal_finished = False
        if self._restart_timer is not None:
            self._restart_timer.cancel()
        self._restart_timer = self.create_timer(self.repeat_delay_sec, self._restart_cb)

    def _restart_cb(self) -> None:
        """Timer callback for looped route restarts."""
        if self._restart_timer is not None:
            self._restart_timer.cancel()
            self._restart_timer = None

        if self.stop_event.is_set() or self._goal_active:
            return

        self._send_route_goal()

    def stop(self) -> None:
        """Stop the route runner."""
        self.stop_event.set()
        if self._restart_timer is not None:
            self._restart_timer.cancel()
            self._restart_timer = None
        if self._start_timer is not None:
            self._start_timer.cancel()


def main(args=None) -> None:
    """Entry point for the route_loop_runner node."""
    rclpy.init(args=args)
    node = RouteLoopRunner()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
