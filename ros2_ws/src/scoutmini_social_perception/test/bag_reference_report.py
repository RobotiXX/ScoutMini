"""Summarize the existing track topic stored in an ACME ROS bag."""

import argparse
from collections import Counter
import json
import statistics

from rclpy.serialization import deserialize_message
import rosbag2_py
from vision_msgs.msg import Detection2DArray


def main() -> None:
    """Read one bag without DDS and print stable reference-track metrics."""
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--scenario', required=True)
    parser.add_argument(
        '--topic',
        default='/tracks_insta360_x4_image_raw_compressed',
    )
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    frames = 0
    frames_with_people = 0
    detections = 0
    observations = Counter()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, Detection2DArray)
        frames += 1
        detections += len(msg.detections)
        frames_with_people += bool(msg.detections)
        observations.update(
            detection.id for detection in msg.detections if detection.id
        )

    counts = list(observations.values())
    print(json.dumps({
        'scenario': args.scenario,
        'frames': frames,
        'frames_with_people': frames_with_people,
        'detections': detections,
        'unique_reference_ids': len(counts),
        'singleton_reference_ids': sum(count == 1 for count in counts),
        'median_observations_per_id': (
            statistics.median(counts) if counts else None
        ),
    }, sort_keys=True))


if __name__ == '__main__':
    main()
