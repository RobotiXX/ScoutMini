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

### D. Run the actual robot dashboard

```bash
cd ros2_ws
ros2 run scoutmini_display robot_dashboard
```

This is the page-based UI built from the examples. It includes:
- a Go to Room page that sends a Nav2 goal using the room/waypoint name,
- a Stream page that subscribes only while it is visible,
- a Network page that shows saved NetworkManager profiles,
- a Status page that monitors configured sensor rates, Scout battery, and Jetson power.

The Stream, Network, and Status pages are protected by a 4-digit PIN prompt. The PIN is read from `ROBOT_UI_PIN` in your shell environment and falls back to a default value if it is not set.

To set it from `~/.bashrc`, add:

```bash
# example
export ROBOT_UI_PIN=1234
```

If you want a different PIN for your own development shell, change that value in `~/.bashrc` and reopen the terminal.

## 3) Recommended development path

1. Start from `robot_dashboard.py` for the full UI and `example_no_ros_multi_page.py` for a simpler Qt-only reference.
2. Add new pages in `scoutmini_display/pages/` and export them from `scoutmini_display/pages/__init__.py`.
3. Use the `DashboardBackend` node for Nav2/waypoint lookups and page-level ROS behavior.
4. Keep stream subscriptions on-demand by activating them only when the Stream page is shown.
5. Reuse the `Go to Room` pattern for other Nav2 destinations.

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

## 6) Network page

The multi-page Qt demo also includes a Network page that uses `nmcli` and only shows saved NetworkManager Wi-Fi profiles.

It can be used to reconnect to networks the robot has already joined before.

### Network permission on the Jetson

If connection activation reports `Not authorized to control networking`, install
the included PolicyKit rule once from the repository root. Pass the Linux account
that runs the dashboard if it is different from your current account:

```bash
sudo ./scripts/install_dashboard_network_permissions.sh "$USER"
sudo reboot
```

The installer adds that account to the dedicated `scoutmini-network` group and
allows the group to activate or deactivate existing NetworkManager connections.
It does not grant permission to create or modify system connection profiles. You
can inspect NetworkManager's result after reboot with:

```bash
nmcli general permissions | grep network-control
```

The `network-control` permission should report `yes` in the dashboard user's
session.

## 7) Stream page

The Stream page finds image topics from the active ROS graph and subscribes only while the page is visible. It supports both raw `sensor_msgs/msg/Image` topics and compressed `sensor_msgs/msg/CompressedImage` topics, then decodes them with Qt and scales the image to fit the screen while preserving aspect ratio.

## 8) Status page

The Status page reads health data from `DashboardBackend` and shows:
- configured sensor topic rates, marked failing when measured rate is 10% or more below the expected rate,
- Scout battery voltage and estimated percentage from `/scout_status`,
- Jetson power draw from `jetson-stats` when available on the robot.

Status monitoring is configured in:

```bash
ros2_ws/src/scoutmini_display/config/status_monitor.yaml
```

Update that file for the robot's actual sensor topic names, message types, and expected rates. The default config includes topic-rate monitors for lidar, camera, and IMU. It also includes a commented diagnostics example for drivers that publish useful frequency data on `/diagnostics`.

`jetson-stats` is optional. On a normal Ubuntu development machine without `jtop`, the dashboard keeps running and shows Jetson power as unavailable.

## 9) Next extension ideas

- Replace `std_msgs/String` with your own message types.
- Add a teleop page with speed slider and emergency stop button.
- Add log panel with color-coded status lines.

## 10) Add your own Python GUI script

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
