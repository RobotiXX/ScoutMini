# scoutmini_display

`scoutmini_display` contains small ROS 2 + PyQt examples for people who know basic Python and ROS 2 and want to build a robot GUI quickly.

The package is intentionally simple:
- start from no-ROS Qt examples,
- then connect a ROS 2 subscriber,
- then extend pages/widgets for your own robot topics.

## 1) Build this package

From the repository root, go to the ROS workspace and build only this package:

```bash
cd ros2_ws
colcon build --symlink-install --packages-select scoutmini_display
```

Then source the overlay in every terminal you use for running nodes:

```bash
source install/setup.bash
```

To avoid doing this manually every time, add it to your shell startup file:

```bash
grep -qxF "source ~/repos/ScoutMini/ros2_ws/install/setup.bash" ~/.bashrc || echo "source ~/repos/ScoutMini/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If your workspace path is different, replace `~/repos/ScoutMini/ros2_ws` with your own path.

## 2) Run the examples

### A. Test ROS publisher

Terminal 1:

```bash
cd ros2_ws
ros2 run scoutmini_display example_publisher
```

### B. Run ROS subscriber + display

Terminal 2:

```bash
cd ros2_ws
ros2 run scoutmini_display example_subscriber_display
```

You should see the GUI label update with the latest message from `example_publisher`.

### C. Run Qt-only examples (no ROS required)

```bash
cd ros2_ws
ros2 run scoutmini_display example_no_ros_hello_world
ros2 run scoutmini_display example_no_ros_multi_page
```

## 3) Recommended development path

1. Start from `example_no_ros_multi_page.py` to design your layout and page flow.
2. Add robot state variables (battery, mode, mission state).
3. Add ROS subscribers and keep the latest message in class members.
4. Use a Qt timer to refresh widgets from those class members.
5. Add command buttons and publish ROS messages when clicked.

## 4) Pattern for ROS + Qt in one process

A practical pattern used in this package:
- `rclpy` node handles subscriptions/publishers,
- Qt owns the main event loop,
- a short Qt timer calls `rclpy.spin_once(node, timeout_sec=0.0)`,
- a second Qt timer updates labels and widgets.

This avoids blocking the GUI while still processing ROS callbacks.

## 5) Common pitfalls

- Forgetting `source install/setup.bash` after build.
- Running GUI over SSH without X11 forwarding/desktop access.
- Calling blocking ROS spin APIs from the UI thread.
- Updating widgets directly inside long-running callbacks.

## 6) Next extension ideas

- Replace `std_msgs/String` with your own message types.
- Add a diagnostics page (CPU, battery, network, topics heartbeat).
- Add a teleop page with speed slider and emergency stop button.
- Add log panel with color-coded status lines.

## 7) Add your own Python GUI script

When you create a new file in this package, you need two things:
1. A Python file inside `ros2_ws/src/scoutmini_display/scoutmini_display/`.
2. A matching entry in `setup.py` under `entry_points['console_scripts']`.

Example: create `robot_dashboard.py`

```python
# ros2_ws/src/scoutmini_display/scoutmini_display/robot_dashboard.py
import rclpy


def main(args=None):
	rclpy.init(args=args)
	print("Robot dashboard started")
	rclpy.shutdown()


if __name__ == '__main__':
	main()
```

Then add this line in `setup.py` inside `console_scripts`:

```python
'robot_dashboard = scoutmini_display.robot_dashboard:main',
```

Rebuild and run:

```bash
cd ros2_ws
colcon build --symlink-install --packages-select scoutmini_display
ros2 run scoutmini_display robot_dashboard
```
