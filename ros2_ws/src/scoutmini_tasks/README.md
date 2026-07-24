# ScoutMini Tasks

`scoutmini_tasks` owns robot task execution helpers such as the route loop
runner. Human-facing Slack integration lives in the separate
`scoutmini_slack` package.

## Commands

```bash
ros2 run scoutmini_tasks route_loop_runner
```

## Door-Aware Navigation

Door-aware navigation is handled by two `scoutmini_tasks` executables:

- `door_aware_route_runner`: runs a route through Nav2, checks the planned path for configured door crossings, inserts door waypoints at runtime, and waits for the door to clear before crossing.
- `door_scan_filter`: republishes `/scan` as `/scan_global_filtered` after removing returns near known door geometry. The global costmap can use the filtered scan while the local costmap continues to use the raw scan.

Door geometry is configured per map in `doors.json`, for example:

```text
ros2_ws/src/map_tools/maps/fuse_3rd/doors.json
```

Routes stay unchanged on disk. When `auto_insert_door_waypoints` is enabled, the route runner only inserts door waypoints into its in-memory route for the current run.

### Simulation

Use the combined door-aware sim launch when you want Gazebo, Nav2, the scan filter, door sliders, and the route runner together:

```bash
ros2 launch scoutmini_sim simulation_2d_door_aware.launch.py route_name:=route1 spawn_y:=-5.0 initial_pose_y:=-5.0
```

Useful arguments:

- `route_name`: route YAML name under `map_tools/maps/<map_name>/routes/`.
- `map_name`: map folder used for routes and door definitions. Defaults to `fuse_3rd`.
- `doors_file`: explicit door JSON path if you do not want the map default.
- `auto_insert_door_waypoints`: turns runtime insertion on or off. Defaults to `true`.
- `door_slider` / `door2_slider`: open Gazebo slider GUIs for simulated doors.
- `nav_start_delay_sec`, `spawn_start_delay_sec`, `controller_start_delay_sec`: startup delays for slower Gazebo launches.

The regular 2D sim launch also starts `door_scan_filter` so the global costmap has `/scan_global_filtered` even when the route runner is not launched:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py
```

### Real Robot

The real navigation launch starts `door_scan_filter` by default. Run Nav2 first, then start the door-aware route runner in a separate terminal when you want door behavior:

```bash
ros2 launch scoutmini_nav2 navigation.launch.py map_name:=fuse_3rd
ros2 run scoutmini_tasks door_aware_route_runner --ros-args -p route_name:=route1 -p map_name:=fuse_3rd
```

If you disable `door_scan_filter`, make sure the global costmap is not still configured to subscribe to `/scan_global_filtered`. Otherwise Nav2 will wait on a scan topic that is not being published.

### Door Crossing Detection

Before sending each normal route step, `door_aware_route_runner` asks Nav2 for the planned path to that waypoint. It checks each adjacent pair of path points against the configured door segments. If the planned path crosses a door, it inserts the door's approach and crossing waypoints before the original route step.

Direction is inferred from the path near the crossing. The runner compares the local path vector to the configured `pre -> post` door vector:

- Positive dot product: insert `pre` then `post`.
- Negative dot product: insert `post` then `pre`, with both inserted orientations rotated by 180 degrees.

This lets the same door definition work from either side of the doorway.

### Waiting For A Door

At the first inserted door waypoint, the runner checks LiDAR hits over the configured door geometry. It samples the first 5 scans, uses their median hit count as the closed-door baseline, and treats the door as open when the current hit count drops to half of that baseline. After the open threshold is reached, it waits `door_clear_delay_sec` before sending the crossing waypoint.

After the robot reaches the crossing waypoint, the runner keeps that door temporarily cleared, then allows it to be detected again 5 seconds later.

### Adding Doors

To add a door:

1. Add a door entry to the map's `doors.json`.
2. Define the closed-door segment and the gate/crossing segment.
3. Add matching `pre` and `post` waypoint names and poses to the map waypoints file.
4. Tune the crossing tolerance and LiDAR detection radius for the physical doorway.
5. Run a route that would naturally cross the door; the runner should insert the door waypoints at runtime.

Rebuild after code or launch changes:

```bash
cd /ros2_ws
colcon build --packages-select scoutmini_tasks scoutmini_sim scoutmini_nav2
source install/setup.bash
```
