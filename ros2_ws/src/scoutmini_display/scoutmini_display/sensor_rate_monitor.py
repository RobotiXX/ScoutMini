import rclpy
from rclpy.note import Node
from std_msgs.msg import String
import importlib
import time
import json

class SensorRateMonitor(Node):
        def __init__(self):
                super().__init__('sensor_rate_monitor')

                self.declare_parameter('monitored_topics', ['/high_freq_topic'])

                topics_list = self.get_parameter('monitored_topics').get_value()

                self.monitored_data = {}

                for entry in topics_list:
                        try:
                                topic_name, type_Str, label = entry.split(':')
                                msg_class = self._import_message_type(type_str)

                                self.monitored_data[topic_name] = {
                                        'label': label,
                                        'count': 0,
                                        'last_time': time.time(),
                                        'rate': 0.0,
                                        'status': 'OFFLINE'
                                }

                                self.create_subscription(
                                        msg_class, 
                                        topic_name, 
                                        lambda msg, t=topic_name: self._topic_callback(t), 
                                        10
                                )

                                self.get_logger().info(f"Monitoring topic: {topic_name}")
                        except Exception as e:
                                self.get_logger().error(f"Failed to parse topic entry '{entry}':{e}")
                self.aggregate_pub = self.create_publisher(String, '/robot/sensor_status_aggregated', 10)
                self.timer = self.create_timer(1.0, self._publish_metrics)

        def _import_message_type(self,type_str):
                parts = type_str.split('/')
                pkg = parts[0]
                msg_name = parts[-1]
                module = importlib.import_module(f"{pkg}.msg")
                return getattr(module, msg_name)
        
        def _topic_callback(self,topic_name):
                        if topic_name in self.monitored_data:
                                self.monitored_data[topic_name]['count'] += 1

        def _publish_metrics(self):
                now = time.time()
                aggregate_report = {}

                for topic_name, data in self.monitored_data.items():
                        duration = now - data['last_time']
                        msg_count = data['count']

                        computed_rate = msg_count / duration if duration > 0 else 0.0

                        data['rate'] = round(computed_rate, 1)
                        data['status'] = 'ONLINE' if computed_rate > 0 else 'OFFLINE'
                        data['count'] = 0
                        data['last_time'] = now

                        aggregate_report[topic_name] = {
                                'label': data['label'],
                                'rate': data['rate'],
                                'status': data['status']
                        }
                msg = String()
                msg.data = json.dumps(aggregate_report)
                self.aggregate_pub.publish(msg)

def main(args=None):
        rclpy.init(args=args)
        node = SensorRateMonitor()
        try:
                rclpy.spin(node)
        except KeyboardInterrupt:
                pass
        finally:
                node.destroy_node()
                rclpy.try_shutdown()

if __name__ == '__main__':
        main()