import math
import signal
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class DoorSlider(Node):
    def __init__(self):
        super().__init__('door_slider')
        self.declare_parameter('world_name', 'fuse_3rd')
        self.declare_parameter('model_name', 'door_3300')
        self.declare_parameter('hinge_x', 10.575)
        self.declare_parameter('hinge_y', -12.575)
        self.declare_parameter('hinge_z', 0.0)
        self.declare_parameter('closed_yaw', 0.9792)
        self.declare_parameter('min_angle', -math.pi / 2.0)
        self.declare_parameter('max_angle', math.pi / 2.0)
        self.declare_parameter('initial_angle', 0.0)

        self.world_name = self.get_parameter('world_name').value
        self.model_name = self.get_parameter('model_name').value
        self.hinge_x = float(self.get_parameter('hinge_x').value)
        self.hinge_y = float(self.get_parameter('hinge_y').value)
        self.hinge_z = float(self.get_parameter('hinge_z').value)
        self.closed_yaw = float(self.get_parameter('closed_yaw').value)
        self.min_angle = float(self.get_parameter('min_angle').value)
        self.max_angle = float(self.get_parameter('max_angle').value)
        self.initial_angle = float(self.get_parameter('initial_angle').value)

        self.client = self.create_client(
            SetEntityPose,
            f'/world/{self.world_name}/set_pose',
        )
        self.warned_waiting_for_service = False
        self.pending_angle = None

    def set_angle(self, angle):
        angle = max(self.min_angle, min(self.max_angle, float(angle)))
        yaw = self.closed_yaw + angle

        if not self.client.service_is_ready():
            self.pending_angle = angle
            if not self.warned_waiting_for_service:
                self.get_logger().warning(
                    f'/world/{self.world_name}/set_pose is not available yet; waiting for bridge'
                )
                self.warned_waiting_for_service = True
            return

        self.pending_angle = None
        request = SetEntityPose.Request()
        request.entity = Entity(name=self.model_name, type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = self.hinge_x
        request.pose.position.y = self.hinge_y
        request.pose.position.z = self.hinge_z
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = DoorSlider()

    root = tk.Tk()
    root.title(f'{node.model_name} hinge')
    root.resizable(False, False)

    angle_var = tk.DoubleVar(value=node.initial_angle)
    degrees_var = tk.StringVar()
    pending = {'job': None}
    closing = {'value': False}

    def apply_angle():
        pending['job'] = None
        node.set_angle(angle_var.get())
        rclpy.spin_once(node, timeout_sec=0.0)

    def poll_service():
        if closing['value']:
            return
        if node.pending_angle is not None and node.client.service_is_ready():
            node.set_angle(node.pending_angle)
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(200, poll_service)

    def set_angle(value):
        angle = float(value)
        degrees_var.set(f'{math.degrees(angle):.0f} deg')
        if pending['job'] is not None:
            root.after_cancel(pending['job'])
        pending['job'] = root.after(40, apply_angle)

    frame = ttk.Frame(root, padding=12)
    frame.grid(row=0, column=0, sticky='nsew')

    ttk.Label(frame, text=node.model_name).grid(row=0, column=0, sticky='w')
    ttk.Label(frame, textvariable=degrees_var, width=8).grid(row=0, column=1, sticky='e')

    slider = ttk.Scale(
        frame,
        from_=node.min_angle,
        to=node.max_angle,
        orient='horizontal',
        variable=angle_var,
        command=set_angle,
        length=280,
    )
    slider.grid(row=1, column=0, columnspan=2, pady=(8, 0), sticky='ew')

    def close():
        if closing['value']:
            return
        closing['value'] = True
        if pending['job'] is not None:
            root.after_cancel(pending['job'])
            pending['job'] = None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        root.quit()
        root.destroy()

    def handle_signal(signum, frame):
        root.after(0, close)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    root.protocol('WM_DELETE_WINDOW', close)
    set_angle(node.initial_angle)
    poll_service()
    root.mainloop()


if __name__ == '__main__':
    main()
