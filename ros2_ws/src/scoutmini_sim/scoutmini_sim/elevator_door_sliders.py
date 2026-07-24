"""Single-window controls for all simulated elevator doors."""

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


class ElevatorDoorSliders(Node):
    def __init__(self):
        super().__init__('elevator_door_sliders')
        self.declare_parameter('world_name', 'fuse_3rd')
        self.declare_parameter('model_names', ['elevator_a_door'])
        self.declare_parameter('closed_x', [-7.925])
        self.declare_parameter('closed_y', [-31.325])
        self.declare_parameter('closed_yaw', 2.68962233)
        self.declare_parameter('min_position', 0.0)
        self.declare_parameter('max_position', 1.2)
        self.world_name = str(self.get_parameter('world_name').value)
        self.model_names = list(self.get_parameter('model_names').value)
        self.closed_x = [float(v) for v in self.get_parameter('closed_x').value]
        self.closed_y = [float(v) for v in self.get_parameter('closed_y').value]
        self.closed_yaw = float(self.get_parameter('closed_yaw').value)
        self.min_position = float(self.get_parameter('min_position').value)
        self.max_position = float(self.get_parameter('max_position').value)
        if not (len(self.model_names) == len(self.closed_x) == len(self.closed_y)):
            raise ValueError('model_names, closed_x, and closed_y must have equal lengths')
        self.client = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose'
        )
        self.pending = {}
        self.warned_waiting = False

    def set_position(self, index, position):
        position = max(self.min_position, min(self.max_position, float(position)))
        if not self.client.service_is_ready():
            self.pending[index] = position
            if not self.warned_waiting:
                self.get_logger().warning(
                    f'/world/{self.world_name}/set_pose is not available yet'
                )
                self.warned_waiting = True
            return
        self.pending.pop(index, None)
        request = SetEntityPose.Request()
        request.entity = Entity(name=self.model_names[index], type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = (
            self.closed_x[index] - math.sin(self.closed_yaw) * position
        )
        request.pose.position.y = (
            self.closed_y[index] + math.cos(self.closed_yaw) * position
        )
        request.pose.orientation.z = math.sin(self.closed_yaw / 2.0)
        request.pose.orientation.w = math.cos(self.closed_yaw / 2.0)
        self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorDoorSliders()
    root = tk.Tk()
    root.title('Elevator doors')
    root.resizable(False, False)
    frame = ttk.Frame(root, padding=12)
    frame.grid(row=0, column=0, sticky='nsew')
    jobs, variables, labels = {}, [], []
    closing = {'value': False}

    def apply_position(index):
        jobs.pop(index, None)
        if not closing['value']:
            node.set_position(index, variables[index].get())
            rclpy.spin_once(node, timeout_sec=0.0)

    def queue_position(index, value):
        labels[index].set(f'{float(value):.2f} m')
        if index in jobs:
            root.after_cancel(jobs[index])
        jobs[index] = root.after(40, lambda: apply_position(index))

    for index, model_name in enumerate(node.model_names):
        variable = tk.DoubleVar(value=0.0)
        label = tk.StringVar(value='0.00 m')
        variables.append(variable)
        labels.append(label)
        ttk.Label(frame, text=model_name).grid(
            row=index * 2, column=0, sticky='w', pady=(4, 0)
        )
        ttk.Label(frame, textvariable=label, width=8).grid(
            row=index * 2, column=1, sticky='e', pady=(4, 0)
        )
        ttk.Scale(
            frame,
            from_=node.min_position,
            to=node.max_position,
            orient='horizontal',
            variable=variable,
            command=lambda value, i=index: queue_position(i, value),
            length=320,
        ).grid(row=index * 2 + 1, column=0, columnspan=2, sticky='ew')
        node.set_position(index, 0.0)

    def poll_service():
        if closing['value']:
            return
        if node.client.service_is_ready():
            for index, position in list(node.pending.items()):
                node.set_position(index, position)
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(200, poll_service)

    def close():
        closing['value'] = True

    signal.signal(signal.SIGINT, lambda signum, frame: close())
    signal.signal(signal.SIGTERM, lambda signum, frame: close())
    root.protocol('WM_DELETE_WINDOW', close)
    poll_service()
    try:
        while not closing['value']:
            try:
                root.update()
            except tk.TclError:
                break
            time.sleep(0.02)
    finally:
        for job in jobs.values():
            try:
                root.after_cancel(job)
            except tk.TclError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            root.destroy()
        except tk.TclError:
            pass


if __name__ == '__main__':
    main()
