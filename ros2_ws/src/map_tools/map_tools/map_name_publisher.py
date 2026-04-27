#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import rclpy
from std_msgs.msg import String


class MapNamePublisher(Node):
    def __init__(self) -> None:
        super().__init__('map_name_publisher')

        self.declare_parameter('map_name', 'fuse_3rd')
        self.declare_parameter('map_name_topic', '/map_tools/map_name')

        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.map_name_topic = self.get_parameter('map_name_topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(
            String,
            self.map_name_topic,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.published = False
        self.timer = self.create_timer(0.1, self._publish_once)

    def _publish_once(self) -> None:
        if self.published:
            return

        msg = String()
        msg.data = self.map_name
        self.publisher.publish(msg)
        self.get_logger().info(f'Published map name "{self.map_name}" on {self.map_name_topic}')
        self.published = True
        self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapNamePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()