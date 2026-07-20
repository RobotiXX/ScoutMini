#!/usr/bin/env python3
"""Loads and serves waypoint data via ROS 2 service.

The waypoint_server node subscribes to the /map_name topic to determine the
current map, loads the corresponding waypoint JSON file from map_tools, and
provides a GetWaypointsByName service so that other nodes (e.g., route_loop_runner)
can query waypoint coordinates without managing file I/O themselves.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from map_interfaces.srv import GetWaypointsByName
from rclpy.node import Node
from std_msgs.msg import String


class WaypointServer(Node):
    """ROS 2 node that manages waypoint storage and lookup via a service.

    This node loads waypoint JSON files for the current map (determined via
    /map_name topic subscription) and provides a GetWaypointsByName service interface
    for other nodes to query waypoints by name, eliminating the need for direct
    file I/O across multiple nodes.

    Attributes:
        waypoints (List[Dict]): Currently loaded waypoint list for active map.
        current_map_name (str): Name of the active map.
        waypoint_lock (threading.Lock): Protects concurrent access to waypoints.
    """

    def __init__(self) -> None:
        """Initialize the waypoint server and set up subscriptions/services."""
        super().__init__('waypoint_server')

        self.declare_parameter('initial_map_name', 'fuse_3rd')

        self.current_map_name = (
            self.get_parameter('initial_map_name').get_parameter_value().string_value.strip()
        )
        self.waypoints: List[Dict] = []
        self.waypoint_lock = threading.Lock()

        # Subscribe to /map_name to detect map changes
        self.map_name_subscription = self.create_subscription(
            String, '/map_name', self._on_map_name_changed, qos_profile=10
        )

        # Create service to query waypoints
        self.service = self.create_service(GetWaypointsByName, 'get_waypoints', self._handle_get_waypoints)

        # Load initial map waypoints
        self._load_waypoints_for_map(self.current_map_name)

        self.get_logger().info(f'Waypoint server started for map: {self.current_map_name}')

    def _on_map_name_changed(self, msg: String) -> None:
        """Handle incoming /map_name topic messages and reload waypoints.

        Args:
            msg: String message containing the new map name.
        """
        new_map_name = msg.data.strip()
        with self.waypoint_lock:
            if new_map_name != self.current_map_name:
                self.current_map_name = new_map_name
                self.get_logger().info(f'Map changed to: {new_map_name}')
            self._load_waypoints_for_map(new_map_name)

    def _get_waypoints_file_path(self, map_name: str) -> Path:
        """Resolve the path to the waypoints JSON file for a given map.

        Args:
            map_name: Name of the map (folder under map_tools/maps/).

        Returns:
            Path object pointing to the <map_name>_waypoints.json file.
        """
        package_share = Path(get_package_share_directory('map_tools'))
        return package_share / 'maps' / map_name / f'{map_name}_waypoints.json'

    def _load_waypoints_for_map(self, map_name: str) -> None:
        """Load the waypoint JSON file for the specified map.

        Args:
            map_name: Name of the map to load waypoints for.
        """
        waypoints_path = self._get_waypoints_file_path(map_name)

        if not waypoints_path.exists():
            self.get_logger().warn(f'Waypoints file not found: {waypoints_path}')
            with self.waypoint_lock:
                self.waypoints = []
            return

        try:
            with waypoints_path.open('r', encoding='utf-8') as stream:
                data = json.load(stream)

            if not isinstance(data, list):
                self.get_logger().error(f'Expected waypoints JSON to be a list; got {type(data)}')
                with self.waypoint_lock:
                    self.waypoints = []
                return

            with self.waypoint_lock:
                self.waypoints = data

            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {waypoints_path}')

        except Exception as exc:
            self.get_logger().error(f'Failed to load waypoints from {waypoints_path}: {exc}')
            with self.waypoint_lock:
                self.waypoints = []

    def _handle_get_waypoints(
        self, request: GetWaypointsByName.Request, response: GetWaypointsByName.Response
    ) -> GetWaypointsByName.Response:
        """Service callback to retrieve waypoints by name.

        Args:
            request: GetWaypointsByName.Request with a list of waypoint names.
            response: GetWaypointsByName.Response to populate with results.

        Returns:
            Populated GetWaypointsByName.Response with poses and found names.
        """
        with self.waypoint_lock:
            waypoint_by_name = {
                str(wp.get('name', '')).strip(): wp
                for wp in self.waypoints
                if str(wp.get('name', '')).strip()
            }

        response.poses = []
        response.found_names = []
        self.get_logger().info(f'Handling waypoint query for names: {request.waypoint_names}')

        for waypoint_name in request.waypoint_names:
            waypoint_name = waypoint_name.strip()
            waypoint = waypoint_by_name.get(waypoint_name)

            if waypoint is None:
                self.get_logger().warn(f'Waypoint "{waypoint_name}" not found')
                continue

            # Build PoseStamped from waypoint JSON
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

            response.poses.append(pose)
            response.found_names.append(waypoint_name)
        self.get_logger().info(f'Found {len(response.poses)}/{len(request.waypoint_names)} waypoints for query' )
        return response


def main(args=None) -> None:
    """Entry point for the waypoint_server node."""
    rclpy.init(args=args)
    node = WaypointServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
