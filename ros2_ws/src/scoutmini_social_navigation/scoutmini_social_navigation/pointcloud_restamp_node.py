"""Normalize recorded point-cloud stamps to the replay clock."""

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudRestamper(Node):
    """Replace sensor-uptime stamps that cannot synchronize with bag time."""

    def __init__(self) -> None:
        super().__init__('pointcloud_restamper')
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/analysis/velodyne_points')
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_topic').value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, msg: PointCloud2) -> None:
        output = deepcopy(msg)
        output.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = PointCloudRestamper()
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
