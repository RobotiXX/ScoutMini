#!/usr/bin/env python3
"""Follow a named waypoint route in a loop using Nav2.

The route is defined in YAML as a list of waypoint names. The runner subscribes
to /map_name topic to determine the current map and constructs the route YAML
path accordingly, then queries the waypoint_server service (from map_tools) to
resolve waypoint names to coordinates. This eliminates the need for direct file
I/O of waypoint JSON in this node.
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
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String

import yaml


class RouteLoopRunner(Node):
    """Load a named route and navigate it in a loop using Nav2.

    This node:
    1. Subscribes to /map_name to determine the current map
    2. Loads the YAML route file from map_tools/maps/<map_name>/routes/<route_name>.yaml
    3. Queries the waypoint_server service to resolve waypoint names to PoseStamped objects
    4. Sends a NavigateThroughPoses action to Nav2 and optionally loops the route

    Attributes:
        route_name (str): Name of the route YAML file (without .yaml extension).
        current_map_name (str): Currently active map name from /map_name topic.
        _route_waypoint_names (List[str]): Ordered list of waypoint names in the route.
        _route_poses (List[PoseStamped]): Resolved poses from the waypoint service.
    """

    def __init__(self) -> None:
        """Initialize the route loop runner and set up subscriptions/clients."""
        super().__init__('route_loop_runner')

        self.declare_parameter('route_name', 'route')
        self.declare_parameter('action_name', '/navigate_through_poses')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('loop', True)
        self.declare_parameter('repeat_delay_sec', 1.0)
        self.declare_parameter('start_delay_sec', 0.0)

        # timeout for action server
        self.declare_parameter('wait_for_server_sec', 30.0)

        # allow the route to skip requested waypoints not retrieved from waypoint server
        self.declare_parameter('skip_missing_waypoints', False)

        self.route_name = (
            self.get_parameter('route_name').get_parameter_value().string_value.strip()
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

        # Subscribe to /map_name to detect map changes
        self.map_name_sub = self.create_subscription(
            String, '/map_name', self._map_name_sub_cb, qos_profile=QoSPresetProfiles.TRANSIENT_LOCAL.value
        )

        # Action client for navigate_through_poses
        self._action_client = ActionClient(self, NavigateThroughPoses, self.action_name)

        # Service client for waypoint lookup
        self._waypoint_client = self.create_client(GetWaypointsByName, 'get_waypoints')

        # Internal state for goal tracking
        self._goal_active = False
        self._goal_sent_once = False
        self._start_time = self.get_clock().now()
        self._start_timer = self.create_timer(0.5, self._tick)
        self._restart_timer: Optional[object] = None

        self.get_logger().info(f'Route loop runner initialized for route: {self.route_name}')

    def _map_name_sub_cb(self, msg: String) -> None:
        """Handle incoming /map_name topic and reload route for the new map.

        Args:
            msg: String message containing the map name.
        """
        new_map_name = msg.data.strip()
        if new_map_name != self.current_map_name:
            self.current_map_name = new_map_name
            self.get_logger().info(f'Map changed to: {new_map_name}; reloading route')
            self._load_route_for_map(new_map_name)

    def _get_route_file_path(self, map_name: str) -> Path:
        """Construct the path to the route YAML file.

        Args:
            map_name: Name of the map (folder under map_tools/maps/).

        Returns:
            Path object pointing to the route YAML file.
        """
        package_share = Path(get_package_share_directory('map_tools'))
        return package_share / 'maps' / map_name / 'routes' / f'{self.route_name}.yaml'

    def _load_route_for_map(self, map_name: str) -> None:
        """Load and parse the route YAML file, then resolve waypoint names.

        Args:
            map_name: Name of the map to load the route for.
        """
        route_path = self._get_route_file_path(map_name)

        if not route_path.exists():
            self.get_logger().error(f'Route file not found: {route_path}')
            self._route_waypoint_names = []
            self._route_poses = []
            return

        try:
            with route_path.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}

            if isinstance(data, list):
                data = {'waypoints': data}

            if not isinstance(data, dict):
                raise ValueError(f'Route file must contain a YAML mapping or list: {route_path}')

            # Extract waypoint names from the route config
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

            self._route_waypoint_names = route_names

            # Query the waypoint service to resolve names to poses
            self._resolve_route_poses()

            self.get_logger().info(
                f'Loaded route {route_path} with {len(self._route_waypoint_names)} waypoint(s)'
            )

        except Exception as exc:
            self.get_logger().error(f'Failed to load route from {route_path}: {exc}')
            self._route_waypoint_names = []
            self._route_poses = []

    def _resolve_route_poses(self) -> None:
        """Query the waypoint_server service to resolve waypoint names to poses."""
        if not self._waypoint_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Waypoint service not available')
            self._route_poses = []
            return

        try:
            request = GetWaypointsByName.Request()
            request.waypoint_names = self._route_waypoint_names

            future = self._waypoint_client.call_async(request)
            future.add_done_callback(self._waypoint_service_cb)

        except Exception as exc:
            self.get_logger().error(f'Failed to call waypoint service: {exc}')
            self._route_poses = []

    def _waypoint_service_cb(self, future) -> None:
        """Handle response from waypoint_server service.

        Args:
            future: Service call future with GetWaypointsByName.Response.
        """
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

        except Exception as exc:
            self.get_logger().error(f'Waypoint service call failed: {exc}')
            self._route_poses = []

    def _tick(self) -> None:
        """Main loop tick; checks conditions and sends route goal if ready."""
        if not self.auto_start or not self._route_poses:
            return

        if self._goal_active:
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            elapsed_sec = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
            if elapsed_sec > self.wait_for_server_sec:
                self.get_logger().warn(
                    f'Waiting for {self.action_name} action server exceeded {self.wait_for_server_sec:.1f}s'
                )
            return

        if not self._goal_sent_once:
            elapsed_sec = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
            if elapsed_sec < self.start_delay_sec:
                return
            self._goal_sent_once = True

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
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        """Handle response from goal submission.

        Args:
            future: Action goal response future.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_active = False
            self.get_logger().error('Route goal was rejected by Nav2')
            self._schedule_restart()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        """Handle route goal result from Nav2.

        Args:
            future: Action result future.
        """
        self._goal_active = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to retrieve route result: {exc}')
            self._schedule_restart()
            return

        status = result.status
        self.get_logger().info(f'Route finished with Nav2 status code {status}')

        if self.loop and not self.stop_event.is_set():
            self._schedule_restart()

    def _schedule_restart(self) -> None:
        """Schedule a timer to restart the route loop."""
        if not self.loop or self.stop_event.is_set():
            return

        if self._restart_timer is not None:
            self._restart_timer.cancel()

        self._restart_timer = self.create_timer(self.repeat_delay_sec, self._restart_cb)

    def _restart_cb(self) -> None:
        """Callback for restart timer; sends the route goal again."""
        if self._restart_timer is not None:
            self._restart_timer.cancel()
            self._restart_timer = None

        if self.stop_event.is_set() or self._goal_active:
            return

        self._send_route_goal()

    def stop(self) -> None:
        """Stop the route loop runner."""
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
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
