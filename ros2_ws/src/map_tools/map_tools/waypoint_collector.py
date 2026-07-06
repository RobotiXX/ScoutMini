#!/usr/bin/env python3
"""Collect, edit, visualize, and persist map waypoints from RViz.

This node supports two user workflows:
- Add mode: consume RViz /clicked_point and save named waypoints.
- Interact mode: drag waypoints to move, or click to rename/delete.
"""

import json
import os
from pathlib import Path
import sys
import threading
import time
from typing import Dict, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from interactive_markers import InteractiveMarkerServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class WaypointCollector(Node):
    def __init__(self) -> None:
        """Initialize parameters, runtime state, and ROS interfaces.

        Startup is split into two phases:
        - Constructor: declare subscriptions/timers and in-memory state.
        - _maybe_initialize: wait for map name, then load/publish markers.
        """
        super().__init__('waypoint_collector')

        self.declare_parameter('clicked_topic', '/clicked_point')
        self.declare_parameter('marker_topic', '/map_tools/waypoints')
        self.declare_parameter('map_name', '')
        self.declare_parameter('map_name_topic', '/map_name')
        self.declare_parameter('map_name_wait_timeout_sec', 5.0)
        self.declare_parameter('output_file', '')
        self.declare_parameter('interactive_naming', True)
        self.declare_parameter('name_prefix', 'wp')
        self.declare_parameter('marker_scale', 1.0)
        self.declare_parameter('interactive_marker_server', 'map_tools_waypoint_editor')

        self.clicked_topic = self.get_parameter('clicked_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.raw_map_name = self.get_parameter('map_name').get_parameter_value().string_value.strip()
        self.map_name_topic = self.get_parameter('map_name_topic').get_parameter_value().string_value
        self.map_name_wait_timeout_sec = self.get_parameter('map_name_wait_timeout_sec').get_parameter_value().double_value
        self.raw_output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.interactive_naming = self.get_parameter('interactive_naming').get_parameter_value().bool_value
        self.name_prefix = self.get_parameter('name_prefix').get_parameter_value().string_value
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.interactive_marker_server_name = self.get_parameter('interactive_marker_server').get_parameter_value().string_value

        self.map_name = self.raw_map_name
        self.output_file = ''
        self._initialized = False
        self._startup_time = self.get_clock().now()
        # Shared waypoint state used by the input thread and ROS callbacks.
        self.waypoints: List[Dict] = []
        # Only keep the latest point click while waiting for naming input.
        self.pending_point: Optional[PointStamped] = None
        # Keep only the most recent interactive marker selection request.
        self.pending_selected_waypoint: Optional[int] = None
        # Remember the latest interactive marker click until release confirms it was not a drag.
        self.pending_interactive_waypoint: Optional[int] = None
        # Single-user RViz assumption: only one marker can be dragged at a time.
        self.active_drag_waypoint: Optional[int] = None
        self.stop_event = threading.Event()
        self.waypoint_lock = threading.Lock()
        self.input_thread: Optional[threading.Thread] = None
        self.stdin_is_tty = bool(getattr(__import__('sys').stdin, 'isatty', lambda: False)())
        self.map_name_sub = self.create_subscription(String, self.map_name_topic, self.map_name_cb, self._map_name_qos())
        self.clicked_sub = self.create_subscription(PointStamped, self.clicked_topic, self.clicked_cb, 10)
        self.startup_timer = self.create_timer(0.1, self._maybe_initialize)
        self.interactive_server = InteractiveMarkerServer(self, self.interactive_marker_server_name)

    def _map_name_qos(self) -> QoSProfile:
        """QoS used for latched map-name handoff between launcher and collector."""
        return QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

    def map_name_cb(self, msg: String) -> None:
        """Capture map name from topic unless an explicit parameter was provided."""
        if self.raw_map_name:
            return

        candidate = msg.data.strip()
        if not candidate or candidate == self.map_name:
            return

        self.map_name = candidate
        if not self._initialized:
            self.get_logger().info(f'Received map name from {self.map_name_topic}: {self.map_name}')

    def _maybe_initialize(self) -> None:
        """Finish startup once map name is known (or timeout fallback is reached)."""
        if self._initialized:
            return

        if not self.map_name:
            elapsed_sec = (self.get_clock().now() - self._startup_time).nanoseconds / 1e9
            if elapsed_sec < self.map_name_wait_timeout_sec:
                return

            self.map_name = 'default_map'
            self.get_logger().warn(
                f'No map name received on {self.map_name_topic}; falling back to {self.map_name}'
            )

        self.output_file = self._resolve_output_file(self.raw_output_file)

        marker_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)

        self._load_waypoints()
        self._initialized = True
        self._refresh_interactive_markers()

        self.input_thread = threading.Thread(target=self._consume_points, daemon=True)
        self.input_thread.start()

        self.get_logger().info('=' * 60)
        self.get_logger().info('Waypoint Collector Started')
        self.get_logger().info(f'Listening on topic: {self.clicked_topic}')
        self.get_logger().info(f'Publishing markers to: {self.marker_topic}')
        self.get_logger().info(
            f'Interactive marker topic: /{self.interactive_marker_server_name}/update '
            '(add an "InteractiveMarkers" display in RViz)'
        )
        self.get_logger().info(f'Saving waypoints to: {self.output_file}')
        self.get_logger().info(f'Loaded {len(self.waypoints)} existing waypoint(s)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('📍 INSTRUCTIONS:')
        self.get_logger().info('1. In RViz, select the "Publish Point" tool')
        self.get_logger().info('2. Click on locations in the map to add waypoints')
        if self.interactive_naming:
            self.get_logger().info('3. Enter a name for each waypoint (or press Enter for default)')
        self.get_logger().info('4. Switch to RViz "Interact" tool to drag the red sphere or rotate its heading')
        self.get_logger().info('5. Click the small yellow edit handle to open terminal actions (rename/delete)')
        self.get_logger().info('')

        self.startup_timer.cancel()

    def clicked_cb(self, msg: PointStamped) -> None:
        """Buffer the most recent clicked point from RViz add mode."""
        self.pending_point = msg
        self.get_logger().info(
            f'Clicked point received: x={msg.point.x:.3f}, y={msg.point.y:.3f}, '
            f'z={msg.point.z:.3f}, frame={msg.header.frame_id} [latest point buffered]'
        )

    def _consume_points(self) -> None:
        """Background loop for terminal-driven naming and edit menu handling.

        Priority order:
        1) Process the latest waypoint selection requested by interactive marker clicks.
        2) Process latest buffered clicked point for add-mode waypoint creation.
        """
        while rclpy.ok() and not self.stop_event.is_set():
            selected_index = self.pending_selected_waypoint
            self.pending_selected_waypoint = None
            if selected_index is not None:
                self._handle_selected_waypoint(selected_index)
                continue

            if self.pending_point is None:
                time.sleep(0.2)
                continue

            msg = self.pending_point
            self.pending_point = None

            suggested = self._next_suggested_name()
            name = suggested

            if self.interactive_naming and self.stdin_is_tty:
                try:
                    # Flush stdout to ensure prompt appears
                    sys.stdout.flush()
                    entered = input(f'\n📍 Enter waypoint name [{suggested}]: ').strip()
                    if entered:
                        name = entered
                except EOFError:
                    self.get_logger().info(f'Using default name: {suggested}')
                except Exception as exc:
                    self.get_logger().warn(f'Input error, using default name {suggested}: {exc}')
            else:
                if self.interactive_naming:
                    self.get_logger().info(f'Interactive naming disabled (no TTY). Using name: {suggested}')

            name = self._ensure_unique_name(name)
            self._append_waypoint(msg, name)

    def _handle_selected_waypoint(self, index: int) -> None:
        """Handle terminal actions for a selected waypoint (delete/rename/cancel)."""
        with self.waypoint_lock:
            if index < 0 or index >= len(self.waypoints):
                self.get_logger().warn(f'Selected waypoint index out of range: {index}')
                return
            selected = dict(self.waypoints[index])

        self.get_logger().info(
            f'Selected waypoint [{index}] {selected["name"]} at '
            f'({selected["x"]:.2f}, {selected["y"]:.2f}, {selected["z"]:.2f})'
        )

        if not self.stdin_is_tty:
            self.get_logger().info('No TTY available. Use RViz drag to move or run in a terminal for edit menu.')
            return

        print('\nWaypoint edit options: [d]elete, [r]ename, [c]ancel')
        action = input('Select action [c]: ').strip().lower() or 'c'

        if action == 'd':
            with self.waypoint_lock:
                if index >= len(self.waypoints):
                    return
                removed = self.waypoints.pop(index)
            self._save_waypoints()
            self._refresh_interactive_markers()
            self.get_logger().info(f'Deleted waypoint {removed["name"]}')
            return

        elif action == 'r':
            new_name = input(f'New name [{selected["name"]}]: ').strip()
            if not new_name:
                self.get_logger().info('Rename cancelled (empty name).')
                return
            with self.waypoint_lock:
                if index >= len(self.waypoints):
                    return
                unique_name = self._ensure_unique_name(new_name)
                self.waypoints[index]['name'] = unique_name
            self._save_waypoints()
            self._refresh_interactive_markers()
            self.get_logger().info(f'Renamed waypoint to {unique_name}')
            return
        else:
            self._refresh_interactive_markers()
            self.get_logger().info('Edit cancelled.')

    def _default_output_dir(self) -> Path:
        """Resolve where waypoint files should be stored for the active map.

        Prefer workspace source maps directory during development, then fall back
        to package-share maps directory.
        """
        package_share = Path(get_package_share_directory('map_tools'))
        
        # Try to find src directory by traversing up from install
        # install/map_tools/share/map_tools -> go up 4 levels to workspace root, then to src
        workspace_root = package_share.parent.parent.parent.parent
        src_maps_dir = workspace_root / 'src' / 'map_tools' / 'maps'
        
        if src_maps_dir.exists():
            return src_maps_dir / self._resolved_map_name()
        
        # Fall back to install directory
        return package_share / 'maps' / self._resolved_map_name()

    def _resolved_map_name(self) -> str:
        """Return a filesystem-safe map name used for waypoint filenames."""
        candidate = self.map_name.strip() if self.map_name else ''
        if not candidate:
            candidate = 'default_map'
        candidate = candidate.replace(' ', '_').replace('/', '_')
        return candidate

    def _resolve_output_file(self, raw_output_file: str) -> str:
        """Resolve explicit output path, otherwise derive default map waypoint file."""
        if raw_output_file.strip():
            return os.path.expanduser(os.path.expandvars(raw_output_file.strip()))

        output_dir = self._default_output_dir()

        map_name = self._resolved_map_name()
        return str(output_dir / f'{map_name}_waypoints.json')

    def _append_waypoint(self, msg: PointStamped, name: str) -> None:
        """Append a new waypoint, then persist and refresh all visual outputs."""
        waypoint = {
            'name': name,
            'description': '',
            'frame_id': msg.header.frame_id,
            'x': float(msg.point.x),
            'y': float(msg.point.y),
            'z': float(0.001), # negative z affects map rviz interaction
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0,
            },
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
        }
        with self.waypoint_lock:
            self.waypoints.append(waypoint)
        self._save_waypoints()
        self._refresh_interactive_markers()
        
        # Log with more visual feedback
        self.get_logger().info(
            f'Waypoint #{len(self.waypoints)} saved: "{name}" at ({waypoint["x"]:.2f}, {waypoint["y"]:.2f}) '
            f'in frame "{waypoint["frame_id"]}"'
        )

    def _next_suggested_name(self) -> str:
        """Generate default waypoint name like wp_001, wp_002, ..."""
        index = len(self.waypoints) + 1
        return f'{self.name_prefix}_{index:03d}'

    def _ensure_unique_name(self, name: str) -> str:
        """Ensure waypoint name uniqueness by adding numeric suffix when needed."""
        existing = {wp['name'] for wp in self.waypoints}
        if name not in existing:
            return name

        suffix = 2
        while f'{name}_{suffix}' in existing:
            suffix += 1
        return f'{name}_{suffix}'

    def _apply_waypoint_pose(self, index: int, pose) -> Optional[str]:
        """Copy an interactive marker pose into the waypoint model.

        This stores both position and orientation so the waypoint can be moved
        and rotated in RViz.
        """
        moved_name = None
        with self.waypoint_lock:
            if 0 <= index < len(self.waypoints):
                waypoint = self.waypoints[index]
                waypoint['x'] = float(pose.position.x)
                waypoint['y'] = float(pose.position.y)
                waypoint['z'] = float(pose.position.z)
                waypoint['orientation'] = {
                    'x': float(pose.orientation.x),
                    'y': float(pose.orientation.y),
                    'z': float(pose.orientation.z),
                    'w': float(pose.orientation.w),
                }
                moved_name = waypoint['name']

        return moved_name

    def _ensure_waypoint_orientation(self, waypoint: Dict) -> Dict:
        """Ensure legacy waypoint records always have orientation and description fields."""
        description = waypoint.get('description')
        if not isinstance(description, str):
            description = ''
        waypoint['description'] = description

        orientation = waypoint.get('orientation')
        if not isinstance(orientation, dict):
            orientation = {}

        waypoint['orientation'] = {
            'x': float(orientation.get('x', 0.0)),
            'y': float(orientation.get('y', 0.0)),
            'z': float(orientation.get('z', 0.0)),
            'w': float(orientation.get('w', 1.0)),
        }
        return waypoint

    def _save_waypoints(self) -> None:
        """Persist current waypoints to JSON using a lock-protected snapshot."""
        with self.waypoint_lock:
            waypoints_snapshot = [dict(wp) for wp in self.waypoints]

        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, 'w', encoding='utf-8') as f:
            json.dump(waypoints_snapshot, f, indent=2)

    def _load_waypoints(self) -> None:
        """Load existing waypoint JSON on startup, if present."""
        if not os.path.exists(self.output_file):
            return

        try:
            with open(self.output_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if isinstance(data, list):
                with self.waypoint_lock:
                    self.waypoints = [self._ensure_waypoint_orientation(dict(wp)) for wp in data if isinstance(wp, dict)]
                self.get_logger().info(f'Loaded {len(self.waypoints)} existing waypoint(s).')
        except Exception as exc:
            self.get_logger().warn(f'Failed to load existing waypoints: {exc}')

 
    def _refresh_interactive_markers(self) -> None:
        """Rebuild interactive markers used for click-select and drag-move."""
        with self.waypoint_lock:
            waypoints_snapshot = [dict(wp) for wp in self.waypoints]

        self.interactive_server.clear()

        for i, wp in enumerate(waypoints_snapshot):
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = wp.get('frame_id', 'map')
            int_marker.name = f'waypoint_{i}'
            # int_marker.description = wp['name']
            int_marker.pose.position.x = float(wp['x'])
            int_marker.pose.position.y = float(wp['y'])
            int_marker.pose.position.z = float(wp['z'])
            orientation = wp.get('orientation', {})
            int_marker.pose.orientation.x = float(orientation.get('x', 0.0))
            int_marker.pose.orientation.y = float(orientation.get('y', 0.0))
            int_marker.pose.orientation.z = float(orientation.get('z', 0.0))
            int_marker.pose.orientation.w = float(orientation.get('w', 1.0))
            int_marker.scale = max(1.0, self.marker_scale * 2.0)

            # White text for waypoint name

            label = Marker()
            label.type = Marker.TEXT_VIEW_FACING
            label.scale.z = 1.00
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.pose.position.y = -self.marker_scale*0.8 
            label.text = wp['name']
            label.pose

            label_control = InteractiveMarkerControl()
            label_control.name = 'label'
            label_control.interaction_mode = InteractiveMarkerControl.NONE
            label_control.always_visible = True
            label_control.markers.append(label)
            int_marker.controls.append(label_control)


            # Red sphere for visualizing waypoint and adjusting position
            sphere = Marker()
            sphere.type = Marker.SPHERE
            sphere.scale.x = self.marker_scale
            sphere.scale.y = self.marker_scale
            sphere.scale.z = self.marker_scale
            sphere.color.r = 1.0
            sphere.color.g = 0.2
            sphere.color.b = 0.2
            sphere.color.a = 1.0



            move_control = InteractiveMarkerControl()
            move_control.name = 'move_plane'
            move_control.orientation.w = 1.0
            move_control.orientation.x = 0.0
            move_control.orientation.y = 1.0
            move_control.orientation.z = 0.0
            move_control.always_visible = True
            move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
            move_control.markers.append(sphere)



            # Blue flat cylinder around waypoint for adjusting waypoint orientation
            rotate_disk = Marker()
            rotate_disk.type = Marker.CYLINDER
            rotate_disk.scale.x = self.marker_scale * 2.2
            rotate_disk.scale.y = self.marker_scale * 2.2
            rotate_disk.scale.z = 0.02
            rotate_disk.pose.position.z = -0.01
            rotate_disk.color.r = 0.2
            rotate_disk.color.g = 0.6
            rotate_disk.color.b = 1.0
            rotate_disk.color.a = 0.30

            # Blue arrow for visualizing orientation
            heading = Marker()
            heading.type = Marker.ARROW
            heading.scale.x = self.marker_scale * 1.25
            heading.scale.y = self.marker_scale * 0.20
            heading.scale.z = self.marker_scale * 0.20
            heading.pose.position.z = 0.05
            heading.color.r = 0.2
            heading.color.g = 0.6
            heading.color.b = 1.0
            heading.color.a = 0.90

            rotate_control = InteractiveMarkerControl()
            rotate_control.name = 'rotate_plane'
            rotate_control.orientation.w = 1.0
            rotate_control.orientation.x = 0.0
            rotate_control.orientation.y = 1.0
            rotate_control.orientation.z = 0.0
            rotate_control.always_visible = True
            rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            
            rotate_control.markers.append(heading)
            rotate_control.markers.append(rotate_disk)

            int_marker.controls.append(move_control)
            int_marker.controls.append(rotate_control)
            
            # Yellow cube for CLI editing menu
            edit_button = Marker()
            edit_button.type = Marker.CUBE
            relative_cube_scale = self.marker_scale * 0.3
            edit_button.scale.x = relative_cube_scale
            edit_button.scale.y = relative_cube_scale
            edit_button.scale.z = relative_cube_scale
            edit_button.pose.position.z = self.marker_scale*1
            edit_button.color.r = 1.0
            edit_button.color.g = 0.85
            edit_button.color.b = 0.15
            edit_button.color.a = 1.0
            edit_control = InteractiveMarkerControl()
            edit_control.name = 'edit'
            edit_control.interaction_mode = InteractiveMarkerControl.BUTTON
            edit_control.always_visible = True
            edit_control.markers.append(edit_button)
            int_marker.controls.append(edit_control)

            self.interactive_server.insert(int_marker, feedback_callback=self._interactive_feedback_cb)

        self.interactive_server.applyChanges()

    def _interactive_feedback_cb(self, feedback: InteractiveMarkerFeedback) -> None:
        """Handle interactive marker events.

        BUTTON_CLICK requests terminal edit handling, KEEP_ALIVE/POSE_UPDATE mark
        the marker as being dragged, and MOUSE_UP commits the move to disk.
        """
        if not feedback.marker_name.startswith('waypoint_'):
            return

        try:
            index = int(feedback.marker_name.split('_')[1])
        except (IndexError, ValueError):
            return

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Only the dedicated edit button opens the edit menu.
            if feedback.control_name == 'edit':
                self.pending_point = None
                self.pending_interactive_waypoint = index
            return

        if feedback.event_type in (InteractiveMarkerFeedback.KEEP_ALIVE, InteractiveMarkerFeedback.POSE_UPDATE):
            # KEEP_ALIVE and POSE_UPDATE both indicate an active drag gesture.
            self.active_drag_waypoint = index
            if self.pending_interactive_waypoint == index:
                self.pending_interactive_waypoint = None
            moved_name = self._apply_waypoint_pose(index, feedback.pose)

            if moved_name is not None:
                return

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Drag finished; with single-user RViz we track only one active drag index.
            was_dragging = self.active_drag_waypoint == index
            self.active_drag_waypoint = None
            moved_name = self._apply_waypoint_pose(index, feedback.pose)

            if moved_name is None:
                return

            if was_dragging:
                self._save_waypoints()
                self.get_logger().info(
                    f'Moved waypoint {moved_name} to '
                    f'({feedback.pose.position.x:.3f}, {feedback.pose.position.y:.3f}, {feedback.pose.position.z:.3f}) '
                    f'with orientation '
                    f'({feedback.pose.orientation.x:.3f}, {feedback.pose.orientation.y:.3f}, '
                    f'{feedback.pose.orientation.z:.3f}, {feedback.pose.orientation.w:.3f})'
                )
                return

            if self.pending_interactive_waypoint == index:
                self.pending_interactive_waypoint = None
                self.pending_selected_waypoint = index

    def stop(self) -> None:
        """Stop worker thread and clear interactive marker resources."""
        self.stop_event.set()
        if self.input_thread and self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)
        self.interactive_server.clear()
        self.interactive_server.applyChanges()


def main(args=None) -> None:
    """ROS 2 entrypoint for waypoint collector."""
    rclpy.init(args=args)
    node = WaypointCollector()

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
