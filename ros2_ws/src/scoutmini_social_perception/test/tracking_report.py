"""Summarize detector output during a bounded ROS bag replay."""

import argparse
from collections import Counter
import json
import statistics
import time

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2DArray


class TrackingReport(Node):
    """Collect typed-track and detector latency statistics."""

    def __init__(self) -> None:
        super().__init__('tracking_report')
        self.frames = 0
        self.frames_with_people = 0
        self.detections = 0
        self.track_observations = Counter()
        self.latencies_ms = []
        self.first_stamp = None
        self.last_stamp = None
        self.create_subscription(
            Detection2DArray,
            '/people/tracks_2d',
            self._tracks_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray,
            '/people/detector_diagnostics',
            self._diagnostics_callback,
            10,
        )

    def _tracks_callback(self, msg: Detection2DArray) -> None:
        self.frames += 1
        self.detections += len(msg.detections)
        if msg.detections:
            self.frames_with_people += 1
        self.track_observations.update(
            detection.id for detection in msg.detections if detection.id
        )
        stamp = float(msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9
        self.first_stamp = stamp if self.first_stamp is None else self.first_stamp
        self.last_stamp = stamp

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            values = {item.key: item.value for item in status.values}
            if 'elapsed_ms' in values:
                self.latencies_ms.append(float(values['elapsed_ms']))

    def summary(self, name: str) -> dict:
        """Return stable JSON-ready report fields."""
        latency = self.latencies_ms
        sorted_latency = sorted(latency)
        p95_index = max(0, int(len(sorted_latency) * 0.95) - 1)
        observations = list(self.track_observations.values())
        source_duration = 0.0
        if self.first_stamp is not None and self.last_stamp is not None:
            source_duration = max(0.0, self.last_stamp - self.first_stamp)
        return {
            'scenario': name,
            'frames': self.frames,
            'frames_with_people': self.frames_with_people,
            'detections': self.detections,
            'unique_public_ids': len(observations),
            'singleton_public_ids': sum(
                count == 1 for count in observations
            ),
            'median_observations_per_id': (
                statistics.median(observations) if observations else None
            ),
            'source_duration_sec': round(source_duration, 3),
            'mean_inference_ms': (
                round(statistics.fmean(latency), 3) if latency else None
            ),
            'p95_inference_ms': (
                round(sorted_latency[p95_index], 3) if latency else None
            ),
        }


def main() -> None:
    """Collect for the requested wall duration and print one JSON object."""
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario')
    parser.add_argument('--duration', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    report = TrackingReport()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(report, timeout_sec=0.1)
        print(json.dumps(report.summary(args.scenario), sort_keys=True))
    finally:
        report.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
