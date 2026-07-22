"""Publish replay-safe odometry transforms from recorded Odometry messages."""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdometryTransformPublisher(Node):
    """Broadcast the pose and epoch timestamp carried by recorded odometry."""

    def __init__(self) -> None:
        super().__init__('odometry_transform_publisher')
        self.declare_parameter('odom_topic', '/odom')
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = msg.header.frame_id or 'odom'
        transform.child_frame_id = msg.child_frame_id or 'base_link'
        position = msg.pose.pose.position
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdometryTransformPublisher()
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
