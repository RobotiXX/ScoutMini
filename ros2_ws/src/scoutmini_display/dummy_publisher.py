import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

class HighFreqPublisher(Node):
    def __init__(self):
        super().__init__('high_freq_publisher')
        self.publisher_ = self.create_publisher(Header, '/high_freq_topic', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Started dummy 100 Hz publisher on /high_freq_topic")

    def timer_callback(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = 'dummy_sensor_link'
        self.publisher_.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = HighFreqPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
