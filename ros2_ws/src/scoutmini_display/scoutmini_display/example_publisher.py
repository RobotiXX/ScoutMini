#!/home/nle/repos/ScoutMini/ros2_ws/qt_venv/bin/python3
"""Minimal ROS 2 publisher used to test ScoutMini GUI subscribers.

Run this node in one terminal, then run the display subscriber in another
terminal. The display should show the latest message published on the topic.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

TOPIC_NAME = 'topic'
PUBLISH_PERIOD = 0.5  # seconds


class MinimalPublisher(Node):
    """Publish incrementing test messages for UI development."""

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, TOPIC_NAME, 10)

        # A ROS timer triggers at fixed rate and calls timer_callback.
        self.timer = self.create_timer(PUBLISH_PERIOD, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        """Build and publish one message each time the timer fires."""
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    """Entry point for `ros2 run scoutmini_display example_publisher`."""
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()