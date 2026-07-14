#!/usr/bin/env python3
#import ros2 python libraries
import rclpy
from rclpy.node import Node

#This helps publish goal pose using Rviz when clicking "2D Goal Pose"
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class ClickToNav(Node):
    """
    Ros 2 Node that allows the user to click a point in Rviz
    and automatically sends the robot to that location
    """
    def __init__(self):
        super().__init__('click_to_nav')

        #Subscribe to Rviz goal clicks
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        #Creates a connection to Nav2 telling
        #it to drive the robot to the specific coordinate
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.get_logger().info("Click To Nav started")
        # terminal should print "Click to Nav started" if successful       

    def goal_callback(self, msg):
        
        # Whenever Rviz2 publishes a new goal, 
        # goal information is published in the terminal
        self.get_logger().info(
            f"New goal received: "
            f"x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}"
        )

        # Send this goal to Nav2
        self.send_goal(msg)

    #Send the recieved goal to Nav2
    def send_goal(self, pose):

        # Make sure Nav2 is running
        if not self.nav_client.wait_for_server(timeout_sec=5):

            self.get_logger().error(
                "Nav2 is not available!"
            )

            return


        # Create a NavigateToPose goal
        goal = NavigateToPose.Goal()

        # Put the RViz clicked position into the goal
        goal.pose = pose


        # Send the goal asynchronously
        future = self.nav_client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )

    #Helps determine whether Nav2 accepeted or rejected the goal
    def goal_response_callback(self, future):

        goal_handle = future.result()


        if not goal_handle.accepted:

            self.get_logger().error(
                "Nav2 rejected the goal"
            )

            return


        self.get_logger().info(
            "Goal accepted. Robot moving."
        )


        # Wait for Nav2 to finish
        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )



    
    # Called when the robot finishes navigating
    def result_callback(self, future):

        result = future.result()


        self.get_logger().info(
            f"Navigation complete. Status: {result.status}"
        )

        self.get_logger().info(
            "Waiting for next RViz click..."
        )
    
    def main(args=None):
        rclpy.init(args=args)

        node = ClickToNav()

        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()