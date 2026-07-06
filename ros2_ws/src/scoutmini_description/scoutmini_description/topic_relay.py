import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ScoutMiniTopicRelay(Node):
    def __init__(self):
        super().__init__('scoutmini_topic_relay')
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)

    def cmd_callback(self, msg):
        stamped = TwistStamped()
        # Jazzy's diff_drive_controller treats a zero stamp as current controller time.
        # This avoids command timeout loops from relay/controller clock skew in simulation.
        stamped.header.frame_id = 'base_footprint'
        stamped.twist = msg
        self.cmd_pub.publish(stamped)

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScoutMiniTopicRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
