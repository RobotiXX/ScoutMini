#!/usr/bin/env python3

"""
Waypoint Navigator

Flow:
1. Receives a waypoint name from RViz through /selected_waypoint
2. Requests the waypoint position from waypoint_server
3. Sends that position to Nav2
4. Waits until Nav2 finishes

This allows:
Click waypoint -> robot goes there -> waits
Click another waypoint -> robot goes there
"""

import rclpy
import threading 

from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

from map_interfaces.srv import GetWaypointsByName


class WaypointNavigator(Node):

    #Initializes the waypoint navigator node
    #Creates the subscriber, service client and action client needed
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Start a separate thread to listen for terminal input
        input_thread = threading.Thread(target=self.terminal_input_loop)
        input_thread.daemon = True
        input_thread.start()

        # Receive waypoint clicks from waypoint_collector
        self.selected_waypoint_sub = self.create_subscription(
            String,
            '/selected_waypoint',
            self.selected_waypoint_callback,
            10
        )

        # Ask waypoint_server for waypoint coordinates
        self.waypoint_client = self.create_client(
            GetWaypointsByName,
            'get_waypoints'
        )

        # Send navigation goals to Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.get_logger().info(
            "Waypoint Navigator Started"
        )
    

    # Recieves a waypoint name from Rviz when the user clicks a waypoint
    # Gets its precise location 
    def selected_waypoint_callback(self, msg):
        """
        Called whenever a waypoint is clicked in RViz.
        """

        waypoint_name = msg.data

        self.get_logger().info(
            f"Received waypoint: {waypoint_name}"
        )

        self.get_waypoint_pose(waypoint_name)

    # Converts waypoint name to PoseStamped position that Nav2 can use
    def get_waypoint_pose(self, waypoint_name):
        """
        Ask waypoint_server for the waypoint coordinates.
        """

        if not self.waypoint_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "Waypoint server not available"
            )
            return

        request = GetWaypointsByName.Request()

        request.waypoint_names = [
            waypoint_name
        ]

        future = self.waypoint_client.call_async(request)

        future.add_done_callback(
            self.waypoint_response_callback
        )
    # Continuously waits for waypoint names entered in the terminal.
    # When a valid waypoint name is entered, it requests that waypoint
    # from the Waypoint Server.
    def terminal_input_loop(self):

        while rclpy.ok():

            waypoint_name = input(
                "\nEnter waypoint name (or 'q' to quit): "
            ).strip()

            if waypoint_name.lower() == "q":
                break

            if waypoint_name == "":
                continue

            self.get_logger().info(
                f"Requesting waypoint: {waypoint_name}"
            )

            request = GetWaypointsByName.Request()
            request.waypoint_names = [waypoint_name]

            future = self.waypoint_client.call_async(request)
            future.add_done_callback(
                self.waypoint_response_callback
            )
    # Extracts the waypoint pose and sends it to Nav2 for navigation
    def waypoint_response_callback(self, future):
        """
        Receive waypoint coordinates.
        """

        self.get_logger().info(
            "Waypoint response callback triggered"
        )
        
        try:
            response = future.result()

        except Exception as e:
            self.get_logger().error(
                f"Waypoint service failed: {e}"
            )
            return


        if len(response.poses) == 0:
            self.get_logger().error(
                "Waypoint not found"
            )
            return


        pose = response.poses[0]

        self.get_logger().info(
            "Waypoint found, sending Nav2 goal"
        )

        self.send_navigation_goal(pose)

    # Sends the navigation goal to Nav2
    def send_navigation_goal(self, pose):
        """
        Send waypoint position to Nav2.
        """

        if not self.nav_client.wait_for_server(timeout_sec=5.0):

            self.get_logger().error(
                "Nav2 navigate_to_pose action not available"
            )

            return


        goal = NavigateToPose.Goal()

        goal.pose = pose


        future = self.nav_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    #Checks whether Nav2 accepeted the navigation goal
    def goal_response_callback(self, future):
        """
        Check if Nav2 accepted the goal.
        """

        goal_handle = future.result()


        if not goal_handle.accepted:

            self.get_logger().error(
                "Nav2 rejected goal"
            )

            return


        self.get_logger().info(
            "Nav2 accepted goal"
        )


        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_finished_callback
        )

    # handles the result after Nav2
    # determines if it is successful or not
    def navigation_finished_callback(self, future):
        """
        Called when Nav2 finishes.
        """

        result = future.result()

        self.get_logger().info(
            f"Navigation finished with status: {result.status}"
        )


# starts the ros2 node and creates the waypoint navigator
def main(args=None):

    rclpy.init(args=args)

    node = WaypointNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()