"""Explicitly armed, time-bounded relay for supervised robot tests."""

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool

from .motion_gate import MotionGate


class MotionGateNode(Node):
    """Relay fresh shadow commands only during a short armed interval."""

    def __init__(self) -> None:
        super().__init__('supervised_motion_gate')
        self.declare_parameter('input_topic', '/adascore/shadow/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter(
            'diagnostics_topic', '/adascore/motion_gate/diagnostics',
        )
        self.declare_parameter(
            'enable_service', '/adascore/motion_gate/enable',
        )
        self.declare_parameter('max_linear_mps', 0.05)
        self.declare_parameter('max_angular_radps', 0.15)
        self.declare_parameter('input_timeout_sec', 0.25)
        self.declare_parameter('max_enable_duration_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 50.0)

        self._gate = MotionGate(
            float(self.get_parameter('max_linear_mps').value),
            float(self.get_parameter('max_angular_radps').value),
            float(self.get_parameter('input_timeout_sec').value),
            float(self.get_parameter('max_enable_duration_sec').value),
        )
        self._output = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10,
        )
        self._diagnostics = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_topic').value),
            self._command_callback,
            10,
        )
        self.create_service(
            SetBool,
            str(self.get_parameter('enable_service').value),
            self._enable_callback,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'Supervised motion gate started DISARMED; output is zero'
        )

    def _now_sec(self) -> float:
        return time.monotonic()

    def _command_callback(self, msg: Twist) -> None:
        if not self._gate.observe(msg.linear.x, msg.angular.z, self._now_sec()):
            self.get_logger().error('Rejected non-finite shadow command')

    def _enable_callback(self, request, response):
        if request.data:
            self._gate.arm(self._now_sec())
            response.message = (
                'armed for '
                f'{self.get_parameter("max_enable_duration_sec").value} s'
            )
        else:
            self._gate.disarm()
            response.message = 'disarmed'
        self._publish_stop()
        response.success = True
        self.get_logger().warning(response.message)
        return response

    def _publish(self) -> None:
        decision = self._gate.evaluate(self._now_sec())
        command = Twist()
        command.linear.x = decision.linear
        command.angular.z = decision.angular
        self._output.publish(command)

        status = DiagnosticStatus()
        status.level = (
            DiagnosticStatus.OK
            if decision.state in ('disarmed', 'forwarding bounded command')
            else DiagnosticStatus.WARN
        )
        status.name = 'scoutmini_social_navigation/supervised_motion_gate'
        status.hardware_id = 'scoutmini_base'
        status.message = decision.state
        status.values = [
            KeyValue(key='armed', value=str(decision.armed).lower()),
            KeyValue(key='linear_mps', value=f'{decision.linear:.3f}'),
            KeyValue(key='angular_radps', value=f'{decision.angular:.3f}'),
            KeyValue(
                key='input_age_sec', value=f'{decision.input_age_sec:.3f}',
            ),
            KeyValue(
                key='remaining_sec', value=f'{decision.remaining_sec:.3f}',
            ),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics.publish(message)

    def _publish_stop(self) -> None:
        self._output.publish(Twist())

    def destroy_node(self):
        self._gate.disarm()
        for _ in range(3):
            self._publish_stop()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the supervised motion gate."""
    rclpy.init(args=args)
    node = MotionGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
