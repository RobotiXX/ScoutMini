"""Publish diagnostics when AdaSCoRe shadow output stalls under live input."""

from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from people_msgs.msg import People
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .shadow_health import ShadowHealth


class ShadowHealthMonitor(Node):
    """Monitor the isolated controller without affecting its data flow."""

    def __init__(self) -> None:
        super().__init__('shadow_health_monitor')
        self.declare_parameter('people_topic', '/adascore/shadow/people')
        self.declare_parameter('output_topic', '/adascore/shadow/cmd_vel')
        self.declare_parameter(
            'action_status_topic',
            '/adascore_shadow/follow_path/_action/status',
        )
        self.declare_parameter('diagnostics_topic', '/adascore/shadow/diagnostics')
        self.declare_parameter('people_timeout_sec', 0.75)
        self.declare_parameter('output_timeout_sec', 1.0)
        self._health = ShadowHealth()
        self.create_subscription(
            People,
            str(self.get_parameter('people_topic').value),
            self._people_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('output_topic').value),
            self._output_callback,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter('action_status_topic').value),
            self._action_callback,
            10,
        )
        self._publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )
        self.create_timer(0.2, self._publish)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _people_callback(self, _msg: People) -> None:
        self._health.observe_people(self._now_sec())

    def _output_callback(self, _msg: Twist) -> None:
        self._health.observe_output(self._now_sec())

    def _action_callback(self, msg: GoalStatusArray) -> None:
        active = any(
            status.status == GoalStatus.STATUS_EXECUTING
            for status in msg.status_list
        )
        self._health.observe_action(active)

    def _publish(self) -> None:
        level, reason, people_age, output_age = self._health.evaluate(
            self._now_sec(),
            float(self.get_parameter('people_timeout_sec').value),
            float(self.get_parameter('output_timeout_sec').value),
        )
        status = DiagnosticStatus()
        status.level = (
            DiagnosticStatus.OK,
            DiagnosticStatus.WARN,
            DiagnosticStatus.ERROR,
        )[level]
        status.name = 'scoutmini_social_navigation/adascore_shadow'
        status.hardware_id = 'adascore_shadow'
        status.message = reason
        status.values = [
            KeyValue(key='people_age_sec', value=f'{people_age:.3f}'),
            KeyValue(key='output_age_sec', value=f'{output_age:.3f}'),
            KeyValue(key='people_messages', value=str(self._health.people_messages)),
            KeyValue(key='output_messages', value=str(self._health.output_messages)),
            KeyValue(
                key='controller_active',
                value=str(self._health.controller_active).lower(),
            ),
            KeyValue(key='stale_transitions', value=str(self._health.stale_transitions)),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._publisher.publish(message)


def main(args=None) -> None:
    """Run the shadow health monitor."""
    rclpy.init(args=args)
    node = ShadowHealthMonitor()
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
