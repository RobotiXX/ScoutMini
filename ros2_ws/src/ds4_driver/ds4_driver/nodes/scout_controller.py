#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class PS4Teleop(Node):
    def __init__(self):
        super().__init__('ps4_teleop')
        # Publishers for twist and additional commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.traj_marking_pub = self.create_publisher(Int8, 'trajectory_marking', 10)
        self.sync_pub = self.create_publisher(Int8, 'sync_command', 10)

        # Subscribe to the Joy messages (published by ds4drv via joy node)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.subscription  # prevent unused variable warning

        # For edge detection to publish on button press (not on every callback)
        self.prev_buttons = []

        # Scaling factors for velocities (tweak as needed)
        self.linear_scale_slow = 0.7
        self.linear_scale_fast = 1.0
        self.angular_scale = 1.0

        # Threshold for trigger activation (assuming triggers are axes in [0,1])
        self.trigger_threshold = 0.001

        # Mapping assumptions (may vary with ds4drv configuration):
        # Axes:
        #   right stick horizontal: index 2
        #   right stick vertical:   index 3
        #   L2 trigger:             index 4
        #   R2 trigger:             index 5
        #
        # Buttons (using typical DS4 layout):
        #   Square:   index 0
        #   X:        index 1
        #   Circle:   index 2
        #   Triangle: index 3

    def joy_callback(self, msg: Joy):
        # Initialize previous buttons state if empty
        if not self.prev_buttons or len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = [0] * len(msg.buttons)

        # Safety: Only allow motion if either L2 or R2 is pressed (trigger value above threshold)
        safety_active = False
        fast_mode = False
        if len(msg.axes) > 5:  # Ensure there are enough axes
            if msg.axes[4] > self.trigger_threshold:
                safety_active = True
                fast_mode = False
            elif msg.axes[5] > self.trigger_threshold:
                safety_active = True
                fast_mode = True

        twist = Twist()
        if safety_active:
            # Map right stick to movement:
            # Assuming: 
            #   - msg.axes[1] is left stick vertical (for linear.x)
            #   - msg.axes[2] is right stick horizontal (for angular.z)
            if fast_mode:
                twist.linear.x = self.linear_scale_fast * msg.axes[1]
            else:
                twist.linear.x = self.linear_scale_slow * msg.axes[1]
            twist.angular.z = self.angular_scale * msg.axes[2]
        else:
            # Safety: do not move if no trigger is held
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

        # Process button presses with rising edge detection
        # Square button (index 0): mark start of trajectory (send 0)
        if self.is_button_pressed(msg.buttons, 0):
            mark_msg = Int8(data=0)
            self.traj_marking_pub.publish(mark_msg)
            self.get_logger().info("Square pressed: Start of trajectory marked (0)")

        # Circle button (index 2): mark end of trajectory (send 1)
        if self.is_button_pressed(msg.buttons, 2):
            mark_msg = Int8(data=1)
            self.traj_marking_pub.publish(mark_msg)
            self.get_logger().info("Circle pressed: End of trajectory marked (1)")

        # Triangle button (index 3): mark trajectory to be discarded (send 2)
        if self.is_button_pressed(msg.buttons, 3):
            mark_msg = Int8(data=2)
            self.traj_marking_pub.publish(mark_msg)
            self.get_logger().info("X pressed: Trajectory discard marked (2)")

        # X button (index 1): publish to sync command (send 1)
        if self.is_button_pressed(msg.buttons, 1):
            sync_msg = Int8(data=1)
            self.sync_pub.publish(sync_msg)
            self.get_logger().info("Triangle pressed: Sync command sent (1)")

        # Update previous button states for next callback
        self.prev_buttons = list(msg.buttons)

    def is_button_pressed(self, buttons, index):
        """Detect a rising edge (button press) for the given index."""
        # Check bounds first
        if index >= len(buttons) or index >= len(self.prev_buttons):
            return False
        # Return True if button is pressed now and was not pressed before
        return buttons[index] == 1 and self.prev_buttons[index] == 0

def main(args=None):
    rclpy.init(args=args)
    node = PS4Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
