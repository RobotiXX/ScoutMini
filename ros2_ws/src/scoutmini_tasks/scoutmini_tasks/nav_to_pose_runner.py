#!/usr/bin/env python3
"""Send a single Nav2 NavigateToPose goal from launch parameters."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class NavToPoseRunner(Node):
    def __init__(self) -> None:
        super().__init__('nav_to_pose_runner')

        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('start_delay_sec', 10.0)
        self.declare_parameter('wait_for_server_sec', 60.0)

        self.action_name = self.get_parameter('action_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.wait_for_server_sec = float(self.get_parameter('wait_for_server_sec').value)

        self._action_client = ActionClient(self, NavigateToPose, self.action_name)
        self._start_time = self.get_clock().now()
        self._goal_sent = False
        self._timer = self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'NavigateToPose runner initialized for x={self.x}, y={self.y}, yaw={self.yaw}'
        )

    def _tick(self) -> None:
        if self._goal_sent:
            return

        elapsed_sec = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed_sec < self.start_delay_sec:
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            if elapsed_sec > self.wait_for_server_sec:
                self.get_logger().warn(
                    f'Waiting for {self.action_name} action server exceeded '
                    f'{self.wait_for_server_sec:.1f}s'
                )
            return

        self._send_goal()

    def _send_goal(self) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y
        goal.pose.pose.position.z = self.z

        half_yaw = self.yaw * 0.5
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)

        self._goal_sent = True
        self.get_logger().info(
            f'Sending goal to {self.action_name}: x={self.x}, y={self.y}, yaw={self.yaw}'
        )
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal was rejected by Nav2')
            return

        self.get_logger().info('NavigateToPose goal accepted by Nav2')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to retrieve NavigateToPose result: {exc}')
            return

        self.get_logger().info(f'NavigateToPose finished with status code {result.status}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavToPoseRunner()

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
