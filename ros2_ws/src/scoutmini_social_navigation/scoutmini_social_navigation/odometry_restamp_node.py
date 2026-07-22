"""Normalize recorded odometry stamps to the replay clock."""

from copy import deepcopy

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdometryRestamper(Node):
    """Replace sensor-uptime stamps that cannot synchronize with image time."""

    def __init__(self) -> None:
        super().__init__('odometry_restamper')
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/analysis/odom')
        self._publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('input_topic').value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, msg: Odometry) -> None:
        output = deepcopy(msg)
        output.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = OdometryRestamper()
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
