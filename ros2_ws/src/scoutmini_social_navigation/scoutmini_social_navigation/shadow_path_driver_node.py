"""Send one deterministic path to the isolated AdaSCoRe controller."""

from __future__ import annotations

import math

from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ShadowPathDriver(Node):
    """Create a forward-only analysis path from the first recorded pose."""

    def __init__(self) -> None:
        super().__init__('shadow_path_driver')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('action_name', '/adascore_shadow/follow_path')
        self.declare_parameter('path_length_m', 4.0)
        self.declare_parameter('path_step_m', 0.2)
        self.declare_parameter('controller_id', 'FollowPath')
        self._pose = None
        self._sent = False
        self._client = ActionClient(
            self,
            FollowPath,
            str(self.get_parameter('action_name').value),
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.2, self._try_send)

    def _odom_callback(self, msg: Odometry) -> None:
        self._pose = msg

    def _try_send(self) -> None:
        if self._sent or self._pose is None:
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            return

        odom = self._pose
        orientation = odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        length = float(self.get_parameter('path_length_m').value)
        step = max(0.05, float(self.get_parameter('path_step_m').value))
        count = max(2, int(math.ceil(length / step)) + 1)

        goal = FollowPath.Goal()
        goal.path.header = odom.header
        goal.path.header.frame_id = 'odom'
        goal.controller_id = str(self.get_parameter('controller_id').value)
        for index in range(count):
            pose = self._make_pose(odom, yaw, min(length, index * step))
            goal.path.poses.append(pose)

        self._sent = True
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.get_logger().info(
            f'Sent offline shadow path: {length:.1f} m, {count} poses'
        )

    @staticmethod
    def _make_pose(odom: Odometry, yaw: float, distance: float):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header = odom.header
        pose.header.frame_id = 'odom'
        pose.pose.position.x = odom.pose.pose.position.x + distance * math.cos(yaw)
        pose.pose.position.y = odom.pose.pose.position.y + distance * math.sin(yaw)
        pose.pose.orientation = odom.pose.pose.orientation
        return pose

    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._sent = False
            self.get_logger().warning(
                'AdaSCoRe is not active yet; retrying the offline shadow path'
            )
            return
        self.get_logger().info('AdaSCoRe accepted the offline shadow path')


def main() -> None:
    rclpy.init()
    node = ShadowPathDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
