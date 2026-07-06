"""Minimal ROS 2 + PyQt display example for ScoutMini.

This script demonstrates a simple and safe pattern for GUI + ROS 2 integration:
1) Use a ROS node for subscriptions.
2) Use Qt timers to periodically process ROS callbacks and refresh widgets.
"""

import rclpy
import sys

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QVBoxLayout,
    QWidget,
)


from rclpy.node import Node
from std_msgs.msg import String

TOPIC_NAME = 'topic'
ROS_RATE = 10  # Hz
UI_RATE = 10  # Hz


class MinimalSubscriber(Node):
    """Receive text messages and expose the latest message to the UI."""

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            TOPIC_NAME,
            self.listener_callback,
            10,
        )
        # Keep a reference to avoid an "unused variable" warning.
        self.subscription
        self.last_msg = ""

    def listener_callback(self, msg):
        """Store the newest ROS message so the UI can display it."""
        self.last_msg = msg.data


class UIWindow(QWidget):
    """Simple window that polls ROS and updates the label at fixed rates."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Scout Mini UI")
        self.label = QLabel("Waiting for message...")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 32px;")

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # Timer for spinning ROS callbacks
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros_once)
        self.ros_timer.start(int(1000 / ROS_RATE))

        # Timer for updating UI
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_display)
        self.ui_timer.start(int(1000 / UI_RATE))

    def spin_ros_once(self):
        # Non-blocking spin keeps the Qt event loop responsive.
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def update_display(self):
        if self.ros_node.last_msg != "":
            self.label.setText(f"Latest message received: {self.ros_node.last_msg}")


def main(args=None):
    """Entry point for `ros2 run scoutmini_display example_subscriber_display`."""
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    minimal_subscriber = MinimalSubscriber()
    window = UIWindow(minimal_subscriber)
    window.resize(1024, 600)
    window.show()
    exit_code = app.exec()

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()