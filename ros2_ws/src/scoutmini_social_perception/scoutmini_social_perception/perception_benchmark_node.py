"""Lightweight topic-rate monitor for perception pipeline topics."""

from __future__ import annotations

from collections import defaultdict
import json
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PerceptionBenchmark(Node):
    def __init__(self) -> None:
        super().__init__('perception_benchmark')
        self.declare_parameter('topics', ['/people/detections_2d', '/people/tracks_2d', '/people/projected'])
        self.declare_parameter('report_rate_hz', 1.0)

        self.counts: Dict[str, int] = defaultdict(int)
        self.last_counts: Dict[str, int] = defaultdict(int)
        self.last_report = self.get_clock().now()
        self.report_pub = self.create_publisher(String, '/perception/benchmark', 10)

        topics = list(self.get_parameter('topics').value)
        for topic in topics:
            self.create_subscription(String, str(topic), self._callback_factory(str(topic)), 10)

        report_rate_hz = max(0.2, float(self.get_parameter('report_rate_hz').value))
        self.create_timer(1.0 / report_rate_hz, self._report)
        self.get_logger().info(f'Monitoring perception topics: {topics}')

    def _callback_factory(self, topic: str):
        def callback(_: String) -> None:
            self.counts[topic] += 1

        return callback

    def _report(self) -> None:
        now = self.get_clock().now()
        dt = max(1e-6, (now - self.last_report).nanoseconds * 1e-9)
        report = {}
        for topic, count in self.counts.items():
            previous = self.last_counts[topic]
            report[topic] = round((count - previous) / dt, 2)
            self.last_counts[topic] = count
        self.last_report = now

        msg = String()
        msg.data = json.dumps(report, separators=(',', ':'), sort_keys=True)
        self.report_pub.publish(msg)
        if report:
            self.get_logger().info(f'Perception rates Hz: {report}')


def main() -> None:
    rclpy.init()
    node = PerceptionBenchmark()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
