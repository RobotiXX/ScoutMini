"""ROS backend shared by the ScoutMini dashboard UI."""

from collections import deque
import os
from typing import Callable

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from map_interfaces.srv import GetWaypointsByName
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import yaml


StatusCallback = Callable[[str], None]


class DashboardBackend(Node):
    """Handle waypoint lookup and Nav2 goal sending for the dashboard."""

    def __init__(self):
        super().__init__('scoutmini_dashboard_backend')
        self._waypoint_client = self.create_client(GetWaypointsByName, 'get_waypoints')
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._status_config = self._load_status_config()
        self._rate_window_seconds = float(self._status_config.get('rate_monitor', {}).get('window_seconds', 5.0))
        self._stale_after_seconds = float(self._status_config.get('rate_monitor', {}).get('stale_after_seconds', 3.0))
        self._failure_fraction = float(self._status_config.get('rate_monitor', {}).get('failure_fraction', 0.10))
        self._rate_monitors = {}
        self._monitor_subscriptions = []
        self._diagnostics_subscription = None
        self._battery_status = {
            'available': False,
            'topic': self._status_config.get('battery', {}).get('topic', '/scout_status'),
            'voltage': None,
            'percentage': None,
            'last_update_age': None,
            'message': 'Waiting for Scout status.',
        }
        self._battery_last_time = None
        self._battery_subscription = None
        self._jetson_status = {
            'available': False,
            'power_watts': None,
            'message': 'jetson-stats is not available on this system.',
        }
        self._jetson = None
        self._jetson_timer = None

        self._setup_status_monitoring()

    def navigate_to_room(self, room_name: str, status_callback: StatusCallback) -> None:
        """Resolve a room name to a waypoint pose and send a Nav2 goal."""
        room_name = room_name.strip()
        if not room_name:
            status_callback('Enter a room name first.')
            return

        if not self._waypoint_client.wait_for_service(timeout_sec=1.0):
            status_callback('Waypoint service is not available yet.')
            return

        request = GetWaypointsByName.Request()
        request.waypoint_names = [room_name]
        future = self._waypoint_client.call_async(request)
        future.add_done_callback(lambda done_future: self._handle_waypoint_response(done_future, room_name, status_callback))

    def _handle_waypoint_response(self, future, room_name: str, status_callback: StatusCallback) -> None:
        try:
            response = future.result()
        except Exception as exc:
            status_callback(f'Failed to resolve room "{room_name}": {exc}')
            return

        if not response.poses:
            status_callback(f'Room "{room_name}" was not found in the current map.')
            return

        pose = response.poses[0]
        status_callback(f'Sending Nav2 goal to "{room_name}"...')
        self._send_nav_to_pose_goal(room_name, pose, status_callback)

    def _send_nav_to_pose_goal(
        self,
        room_name: str,
        pose: PoseStamped,
        status_callback: StatusCallback,
    ) -> None:
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            status_callback('Nav2 NavigateToPose action server is not available yet.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self._nav_to_pose_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda done_future: self._handle_goal_response(done_future, room_name, status_callback)
        )

    def _handle_goal_response(self, future, room_name: str, status_callback: StatusCallback) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            status_callback(f'Failed to send Nav2 goal for "{room_name}": {exc}')
            return

        if not goal_handle.accepted:
            status_callback(f'Nav2 rejected the goal for "{room_name}".')
            return

        status_callback(f'Nav2 accepted the goal for "{room_name}".')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self._handle_goal_result(done_future, room_name, status_callback)
        )

    def _handle_goal_result(self, future, room_name: str, status_callback: StatusCallback) -> None:
        try:
            result = future.result()
        except Exception as exc:
            status_callback(f'Nav2 result failed for "{room_name}": {exc}')
            return

        status_callback(f'Nav2 finished room "{room_name}" with status {result.status}.')

    def get_status_snapshot(self) -> dict:
        """Return current robot health data for the Qt status page."""
        now = self.get_clock().now()
        sensors = []

        for monitor in self._rate_monitors.values():
            self._prune_monitor_samples(monitor, now)
            rate_hz = self._monitor_rate_hz(monitor, now)
            age = self._age_seconds(monitor.get('last_time'), now)
            expected_rate = float(monitor.get('expected_rate_hz', 0.0))
            minimum_rate = expected_rate * (1.0 - self._failure_fraction)
            stale = age is None or age > self._stale_after_seconds
            failing = stale or (expected_rate > 0.0 and rate_hz < minimum_rate)
            message = 'OK'
            if age is None:
                message = 'No messages received.'
            elif stale:
                message = f'Stale: last update {age:.1f}s ago.'
            elif failing:
                message = f'Below expected rate by at least {self._failure_fraction:.0%}.'

            sensors.append({
                'name': monitor['name'],
                'topic': monitor['topic'],
                'monitor_type': monitor['monitor_type'],
                'expected_rate_hz': expected_rate,
                'rate_hz': rate_hz,
                'last_update_age': age,
                'failing': failing,
                'message': message,
            })

        if self._battery_last_time is not None:
            self._battery_status['last_update_age'] = self._age_seconds(self._battery_last_time, now)

        return {
            'sensors': sensors,
            'battery': dict(self._battery_status),
            'jetson': dict(self._jetson_status),
        }

    def _setup_status_monitoring(self):
        for spec in self._status_config.get('sensor_monitors', []):
            monitor_type = spec.get('monitor_type', 'topic_rate')
            monitor = {
                'name': spec.get('name') or spec.get('topic', 'Unnamed sensor'),
                'monitor_type': monitor_type,
                'topic': spec.get('topic', ''),
                'message_type': spec.get('message_type', ''),
                'expected_rate_hz': float(spec.get('expected_rate_hz', 0.0)),
                'samples': deque(),
                'last_time': None,
                'diagnostic_name': spec.get('diagnostic_name', ''),
                'hardware_id': spec.get('hardware_id', ''),
                'rate_key': spec.get('rate_key', ''),
            }
            if not monitor['topic']:
                self.get_logger().warning(f'Skipping status monitor without topic: {spec!r}')
                continue

            self._rate_monitors[monitor['name']] = monitor
            if monitor_type == 'diagnostics_rate':
                self._create_diagnostics_subscription(monitor['topic'])
            else:
                self._create_rate_subscription(monitor)

        self._create_battery_subscription()
        self._setup_jetson_stats()

    def _create_rate_subscription(self, monitor):
        try:
            msg_type = get_message(monitor['message_type'])
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.get_logger().warning(
                f'Could not load message type {monitor["message_type"]!r} for {monitor["name"]}: {exc}'
            )
            return

        subscription = self.create_subscription(
            msg_type,
            monitor['topic'],
            lambda msg, sensor_name=monitor['name']: self._record_monitor_sample(sensor_name),
            10,
        )
        self._monitor_subscriptions.append(subscription)
        self.get_logger().info(
            f'Status monitor subscribed to {monitor["topic"]} as {monitor["message_type"]}.'
        )

    def _create_diagnostics_subscription(self, topic):
        if self._diagnostics_subscription is not None:
            return

        self._diagnostics_subscription = self.create_subscription(
            DiagnosticArray,
            topic,
            self._diagnostics_callback,
            10,
        )
        self.get_logger().info(f'Status monitor subscribed to diagnostics topic {topic}.')

    def _record_monitor_sample(self, sensor_name):
        monitor = self._rate_monitors.get(sensor_name)
        if monitor is None:
            return
        now = self.get_clock().now()
        monitor['samples'].append(now)
        monitor['last_time'] = now
        self._prune_monitor_samples(monitor, now)

    def _diagnostics_callback(self, msg):
        now = self.get_clock().now()
        for status in msg.status:
            for monitor in self._rate_monitors.values():
                if monitor['monitor_type'] != 'diagnostics_rate':
                    continue
                if monitor['diagnostic_name'] and monitor['diagnostic_name'] not in status.name:
                    continue
                if monitor['hardware_id'] and monitor['hardware_id'] not in status.hardware_id:
                    continue
                reported_rate = self._diagnostic_rate_value(status.values, monitor['rate_key'])
                if reported_rate is None:
                    continue
                monitor['samples'].clear()
                monitor['last_time'] = now
                monitor['diagnostic_rate_hz'] = reported_rate

    def _diagnostic_rate_value(self, values, configured_key):
        for key_value in values:
            if configured_key and configured_key != key_value.key:
                continue
            if configured_key or 'frequency' in key_value.key.lower() or 'rate' in key_value.key.lower():
                try:
                    return float(key_value.value.split()[0])
                except (IndexError, TypeError, ValueError):
                    return None
        return None

    def _prune_monitor_samples(self, monitor, now):
        cutoff = self._time_seconds(now) - self._rate_window_seconds
        while monitor['samples'] and self._time_seconds(monitor['samples'][0]) < cutoff:
            monitor['samples'].popleft()

    def _monitor_rate_hz(self, monitor, now):
        if monitor['monitor_type'] == 'diagnostics_rate':
            return float(monitor.get('diagnostic_rate_hz', 0.0))

        samples = monitor['samples']
        if len(samples) < 2:
            return 0.0

        elapsed = self._time_seconds(samples[-1]) - self._time_seconds(samples[0])
        if elapsed <= 0.0:
            return 0.0

        return max(0.0, (len(samples) - 1) / elapsed)

    def _create_battery_subscription(self):
        battery_config = self._status_config.get('battery', {})
        topic = battery_config.get('topic', '/scout_status')
        message_type_name = battery_config.get('message_type', 'scout_msgs/msg/ScoutStatus')
        try:
            msg_type = get_message(message_type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self._battery_status['message'] = f'Battery message type unavailable: {message_type_name}'
            self.get_logger().warning(f'Could not load battery message type {message_type_name!r}: {exc}')
            return

        self._battery_subscription = self.create_subscription(
            msg_type,
            topic,
            self._battery_callback,
            10,
        )
        self.get_logger().info(f'Subscribed to battery status topic {topic}.')

    def _battery_callback(self, msg):
        battery_config = self._status_config.get('battery', {})
        voltage_field = battery_config.get('voltage_field', 'battery_voltage')
        voltage = getattr(msg, voltage_field, None)
        percentage = getattr(msg, 'battery_percentage', None)

        if percentage is None and voltage is not None:
            empty_voltage = float(battery_config.get('empty_voltage', 21.0))
            full_voltage = float(battery_config.get('full_voltage', 29.4))
            if full_voltage > empty_voltage:
                percentage = (float(voltage) - empty_voltage) / (full_voltage - empty_voltage)
                percentage = min(1.0, max(0.0, percentage))

        self._battery_last_time = self.get_clock().now()
        self._battery_status.update({
            'available': voltage is not None or percentage is not None,
            'voltage': float(voltage) if voltage is not None else None,
            'percentage': float(percentage) if percentage is not None else None,
            'message': 'OK' if voltage is not None or percentage is not None else 'Battery fields not found.',
        })

    def _setup_jetson_stats(self):
        jetson_config = self._status_config.get('jetson', {})
        if not jetson_config.get('enabled', True):
            self._jetson_status['message'] = 'Jetson monitoring disabled in config.'
            return

        try:
            from jtop import jtop
        except ImportError:
            self.get_logger().info('jetson-stats is not installed; Jetson power monitoring disabled.')
            return

        try:
            self._jetson = jtop()
            self._jetson.start()
        except Exception as exc:
            self._jetson = None
            self._jetson_status['message'] = f'jetson-stats unavailable: {exc}'
            self.get_logger().warning(self._jetson_status['message'])
            return

        poll_seconds = float(jetson_config.get('poll_seconds', 2.0))
        self._jetson_timer = self.create_timer(poll_seconds, self._poll_jetson_stats)
        self._jetson_status['message'] = 'Waiting for jetson-stats data.'

    def _poll_jetson_stats(self):
        if self._jetson is None:
            return

        try:
            stats = self._jetson.stats
            power_mw = stats.get('Power TOT') or stats.get('Power VDD_IN')
            if power_mw is None:
                self._jetson_status.update({
                    'available': False,
                    'power_watts': None,
                    'message': 'Jetson power field not found.',
                })
                return

            self._jetson_status.update({
                'available': True,
                'power_watts': float(power_mw) / 1000.0,
                'message': 'OK',
            })
        except Exception as exc:
            self._jetson_status.update({
                'available': False,
                'power_watts': None,
                'message': f'Failed to read jetson-stats: {exc}',
            })

    def _load_status_config(self):
        config_path = os.environ.get('SCOUTMINI_STATUS_CONFIG', '').strip()
        if not config_path:
            try:
                config_path = os.path.join(
                    get_package_share_directory('scoutmini_display'),
                    'config',
                    'status_monitor.yaml',
                )
            except PackageNotFoundError:
                config_path = ''

        source_config_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'config', 'status_monitor.yaml')
        )
        if not config_path or not os.path.exists(config_path):
            config_path = source_config_path

        try:
            with open(config_path, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}
        except OSError as exc:
            self.get_logger().warning(f'Could not read status config {config_path!r}: {exc}')
            return {}

        self.get_logger().info(f'Loaded status monitor config from {config_path}.')
        return config

    def _age_seconds(self, stamp, now):
        if stamp is None:
            return None
        return max(0.0, self._time_seconds(now) - self._time_seconds(stamp))

    def _time_seconds(self, stamp):
        return float(stamp.nanoseconds) / 1_000_000_000.0

    def destroy_node(self):
        if self._jetson is not None:
            try:
                self._jetson.close()
            except Exception as exc:
                self.get_logger().warning(f'Failed to close jetson-stats cleanly: {exc}')
            self._jetson = None
        super().destroy_node()
