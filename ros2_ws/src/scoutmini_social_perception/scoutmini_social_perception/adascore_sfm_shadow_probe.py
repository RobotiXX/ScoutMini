"""Read-only AdaSCoRe SocialForceModel subscription probe."""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node


def main() -> None:
    parser = argparse.ArgumentParser(description='Verify AdaSCoRe SFM receives people_msgs without Nav2 motion.')
    parser.add_argument('--people-topic', default='/adascore/shadow/people')
    parser.add_argument('--agents-config', default='social_nav.yaml')
    parser.add_argument('--duration-sec', type=float, default=10.0)
    parser.add_argument('--min-people', type=int, default=1)
    parser.add_argument('--fail-on-missing', action='store_true')
    args = parser.parse_args()

    rclpy.init(args=['--ros-args', '-r', f'/people:={args.people_topic}'])
    node = Node('adascore_sfm_shadow_probe')
    report = {
        'people_topic': args.people_topic,
        'agents_config': args.agents_config,
        'messages_received': 0,
        'max_people': 0,
        'last_people_count': 0,
        'sfm_last_people_count': 0,
        'sfm_max_people': 0,
        'passed': False,
        'error': '',
    }
    try:
        try:
            from adascore.utils.sfm import SocialForceModel
            from people_msgs.msg import People
        except Exception as exc:  # noqa: BLE001
            report['error'] = f'Failed to import AdaSCoRe SocialForceModel: {exc}'
            print(json.dumps(report, indent=2, sort_keys=True))
            sys.exit(2 if args.fail_on_missing else 0)

        sfm = SocialForceModel(node, args.agents_config)
        observed_people = {'messages': 0, 'max_people': 0, 'last_people_count': 0}

        def shadow_callback(msg: People) -> None:
            people_count = len(msg.people)
            observed_people['messages'] += 1
            observed_people['last_people_count'] = people_count
            observed_people['max_people'] = max(int(observed_people['max_people']), people_count)

        node.create_subscription(People, args.people_topic, shadow_callback, 10)
        start = time.monotonic()
        end = start + max(0.1, args.duration_sec)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            sfm_people_count = len(getattr(sfm.people_msg, 'people', []))
            report['messages_received'] = int(observed_people['messages'])
            report['last_people_count'] = int(observed_people['last_people_count'])
            report['sfm_last_people_count'] = sfm_people_count
            report['sfm_max_people'] = max(int(report['sfm_max_people']), sfm_people_count)
            report['max_people'] = max(int(report['max_people']), int(observed_people['max_people']), sfm_people_count)
            direct_ready = (
                int(observed_people['messages']) > 0
                and int(observed_people['max_people']) >= args.min_people
            )
            sfm_ready = sfm_people_count >= args.min_people
            if direct_ready and sfm_ready:
                report['passed'] = True
                break

        print(json.dumps(report, indent=2, sort_keys=True))
        sys.exit(0 if report['passed'] or not args.fail_on_missing else 2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
