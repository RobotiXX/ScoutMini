import math
import signal
import time
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
        self.declare_parameter('motion_type', 'hinged')
        self.declare_parameter('hinge_x', 10.575)
        self.declare_parameter('hinge_y', -12.575)
        self.declare_parameter('hinge_z', 0.0)
        self.declare_parameter('closed_yaw', 0.9792)
        self.declare_parameter('min_angle', -math.pi / 2.0)
        self.declare_parameter('max_angle', math.pi / 2.0)
        self.declare_parameter('initial_angle', 0.0)
        self.declare_parameter('slide_axis_yaw', 0.0)
        self.declare_parameter('min_position', 0.0)
        self.declare_parameter('max_position', 1.0)
        self.declare_parameter('initial_position', 0.0)

        self.world_name = self.get_parameter('world_name').value
        self.model_name = self.get_parameter('model_name').value
        self.motion_type = str(self.get_parameter('motion_type').value)
        self.hinge_x = float(self.get_parameter('hinge_x').value)
        self.hinge_y = float(self.get_parameter('hinge_y').value)
        self.hinge_z = float(self.get_parameter('hinge_z').value)
        self.closed_yaw = float(self.get_parameter('closed_yaw').value)
        self.min_angle = float(self.get_parameter('min_angle').value)
        self.max_angle = float(self.get_parameter('max_angle').value)
        self.initial_angle = float(self.get_parameter('initial_angle').value)
        self.slide_axis_yaw = float(self.get_parameter('slide_axis_yaw').value)
        self.min_position = float(self.get_parameter('min_position').value)
        self.max_position = float(self.get_parameter('max_position').value)
        self.initial_position = float(self.get_parameter('initial_position').value)

        self.client = self.create_client(
            SetEntityPose,
            f'/world/{self.world_name}/set_pose',
        )
        self.warned_waiting_for_service = False
        self.pending_value = None

    def set_position(self, value):
        if self.motion_type == 'sliding':
            value = max(self.min_position, min(self.max_position, float(value)))
            yaw = self.closed_yaw
            x = self.hinge_x - math.sin(self.slide_axis_yaw) * value
            y = self.hinge_y + math.cos(self.slide_axis_yaw) * value
        else:
            value = max(self.min_angle, min(self.max_angle, float(value)))
            yaw = self.closed_yaw + value
            x = self.hinge_x
            y = self.hinge_y

        if not self.client.service_is_ready():
            self.pending_value = value
            if not self.warned_waiting_for_service:
                self.get_logger().warning(
                    f'/world/{self.world_name}/set_pose is not available yet; waiting for bridge'
                )
                self.warned_waiting_for_service = True
            return

        self.pending_value = None
        request = SetEntityPose.Request()
        request.entity = Entity(name=self.model_name, type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = self.hinge_z
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = DoorSlider()

    root = tk.Tk()
    root.title(f'{node.model_name} {node.motion_type}')
    root.resizable(False, False)

    is_sliding = node.motion_type == 'sliding'
    initial_value = node.initial_position if is_sliding else node.initial_angle
    min_value = node.min_position if is_sliding else node.min_angle
    max_value = node.max_position if is_sliding else node.max_angle
    position_var = tk.DoubleVar(value=initial_value)
    value_label_var = tk.StringVar()
    pending = {'job': None}
    closing = {'value': False}

    def apply_angle():
        pending['job'] = None
        if closing['value']:
            return
        node.set_position(position_var.get())
        rclpy.spin_once(node, timeout_sec=0.0)

    def poll_service():
        if closing['value']:
            return
        if node.pending_value is not None and node.client.service_is_ready():
            node.set_position(node.pending_value)
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(200, poll_service)

    def set_position(value):
        value = float(value)
        if is_sliding:
            value_label_var.set(f'{value:.2f} m')
        else:
            value_label_var.set(f'{math.degrees(value):.0f} deg')
        if pending['job'] is not None:
            root.after_cancel(pending['job'])
        pending['job'] = root.after(40, apply_angle)

    frame = ttk.Frame(root, padding=12)
    frame.grid(row=0, column=0, sticky='nsew')

    ttk.Label(frame, text=node.model_name).grid(row=0, column=0, sticky='w')
    ttk.Label(frame, textvariable=value_label_var, width=8).grid(row=0, column=1, sticky='e')

    slider = ttk.Scale(
        frame,
        from_=min_value,
        to=max_value,
        orient='horizontal',
        variable=position_var,
        command=set_position,
        length=280,
    )
    slider.grid(row=1, column=0, columnspan=2, pady=(8, 0), sticky='ew')

    def close():
        closing['value'] = True

    def handle_signal(signum, frame):
        closing['value'] = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    root.protocol('WM_DELETE_WINDOW', close)
    set_position(initial_value)
    poll_service()

    try:
        while not closing['value']:
            try:
                root.update()
            except tk.TclError:
                closing['value'] = True
                break
            time.sleep(0.02)
    finally:
        if pending['job'] is not None:
            try:
                root.after_cancel(pending['job'])
            except tk.TclError:
                pass
            pending['job'] = None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            root.destroy()
        except tk.TclError:
            pass


if __name__ == '__main__':
    main()
