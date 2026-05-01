import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('use_sim_time', False)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(0.5, self._publish_once)
        self.published = False

    def _publish_once(self):
        if self.published:
            return

        if self.publisher.get_subscription_count() == 0:
            return

        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        yaw = self.get_parameter('yaw').value

        message = PoseWithCovarianceStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.position.z = float(z)

        half_yaw = float(yaw) * 0.5
        message.pose.pose.orientation.z = math.sin(half_yaw)
        message.pose.pose.orientation.w = math.cos(half_yaw)

        message.pose.covariance = [0.0] * 36
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.06853891945200942

        self.publisher.publish(message)
        self.published = True
        self.get_logger().info(
            f'Published initial pose x={x}, y={y}, z={z}, yaw={yaw} on /initialpose'
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()