# map_tools

A ROS 2 package for waypoint collection and map management for the ScoutMini robot.

## Overview

`map_tools` provides tools to collect and visualize waypoints on a map using RViz. The main functionality includes:

- **Waypoint Collector**: A node that listens for clicked points from RViz and saves them to a JSON file
- **Map Server Integration**: Displays maps using Nav2's map_server
- **RViz Visualization**: Shows waypoints as markers on the map

## Quick Start

**Terminal 1** - Start infrastructure (map server, RViz, frame transforms):
```bash
cd ~/repos/ScoutMini/ros2_ws
source install/setup.bash
ros2 launch map_tools waypoint_collector.launch.py \
    map_name:=fuse_3rd \
    use_rviz:=true \
    use_map_server:=true
```

**Terminal 2** - Run waypoint collector with interactive naming support:
```bash
cd ~/repos/ScoutMini/ros2_ws
source install/setup.bash
ros2 run map_tools waypoint_collector
```

**Waypoint Collector 2** - show waypoints and AprilTag anchors together:
```bash
cd ~/repos/ScoutMini/ros2_ws
source install/setup.bash
ros2 run map_tools waypoint_collector_2
```

**AprilTag collection** - start the detector against your ZED2 image stream:
```bash
cd ~/repos/ScoutMini/ros2_ws
source install/setup.bash
ros2 launch map_tools zed_apriltag_36h11.launch.py \
   image_topic:=/zed/zed_node/left/image_rect_color \
   camera_info_topic:=/zed/zed_node/left/camera_info
```

Then, in a separate terminal, run the collector and press Enter whenever the robot is in front of a tag:
```bash
cd ~/repos/ScoutMini/ros2_ws
source install/setup.bash
ros2 run map_tools apriltag_tag_collector
```

## Usage

### Collecting Waypoints

1. **In RViz**:
   - Select the "Publish Point" tool from the toolbar
   - Click on locations on the map where you want to place waypoints

2. **In Terminal 2** (waypoint_collector node):
   - You'll see a prompt asking for the waypoint name
   - Enter a custom name or press Enter to use the default (e.g., `wp_001`, `wp_002`)
   - The waypoint will be saved and appear on the map as a red sphere with a label
   - The node reads the active `map_name` from the latched `/map_tools/map_name` topic published by Terminal 1

### Collecting AprilTag Anchors

1. Launch the detector with `ros2 launch map_tools zed_apriltag_36h11.launch.py`.
2. Make sure `/map_name` is being published for the localized map you are driving on.
3. Run `ros2 run map_tools apriltag_tag_collector` in a terminal.
4. Drive the robot until the tag is centered and stable in view.
5. Press Enter in the collector terminal to buffer the next 10 detections.
6. When the pose is stable enough, enter a human-readable description.
7. The collector writes the tag entry to `src/map_tools/maps/<map_name>/tags.json`.

### Visualizing Tags with Waypoints

Run `ros2 run map_tools waypoint_collector_2` to load the normal waypoint JSON and the map's `tags.json` at the same time. Waypoints stay interactive and editable, while tags appear as blue markers with labels so you can see the collected AprilTag anchors on the same map.

### Viewing Waypoints

- Red spheres with white text labels appear on the map in RViz
- Waypoints are saved to: `src/map_tools/maps/<map_name>/<map_name>_waypoints.json`
- Waypoints are persistent - they load automatically when the node starts

### Editing Waypoints (Interactive Markers)

1. In RViz, make sure the `Waypoint Editor` display is enabled.
2. Switch RViz to the `Interact` tool.
3. Click and drag the blue ring/handle around a waypoint to move it.
4. Release mouse button to save the new position.
5. Click the waypoint marker to open terminal actions.
6. In Terminal 2, choose an action:
   - `d` delete
   - `r` rename
   - `c` cancel

### Map Selection

Change which map is used:

```bash
ros2 launch map_tools waypoint_collector.launch.py map_name:=your_map_name
```

### Route Looping

To drive a loop over named waypoints, create a YAML route file like this:

```yaml
map_name: fuse_3rd
loop: true
repeat_delay_sec: 1.0
waypoints:
   - 3401
   - 3302
   - 3304
```

Then start Nav2 with the route loop enabled:

```bash
ros2 launch scoutmini_nav2 navigation.launch.py \
   map:=/absolute/path/to/fuse_3rd.yaml \
   map_name:=fuse_3rd \
   use_route_loop:=true \
   route_file:=/home/nle/repos/ScoutMini/ros2_ws/src/map_tools/maps/fuse_3rd/routes/route.yaml
```

## Launch File Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_name` | `fuse_3rd` | Map folder name under `maps/` directory |
| `use_map_server` | `true` | Start Nav2 map server |
| `use_rviz` | `true` | Launch RViz with waypoint collector config |
| `publish_map_odom_tf` | `true` | Publish identity transform from map to odom |

## Why Separate Terminal for Interactive Naming?

The waypoint_collector node needs a TTY (terminal) to prompt for waypoint names. When launched through a ROS 2 launch file, the subprocess doesn't have access to stdin/stdout of the parent terminal, so interactive prompts won't work.

**Solution**: Run the node in a separate terminal where it has direct TTY access.

## File Storage

Waypoints are saved to:
```
ros2_ws/src/map_tools/maps/<map_name>/<map_name>_waypoints.json
```

Example for the default map:
```
ros2_ws/src/map_tools/maps/fuse_3rd/fuse_3rd_waypoints.json
```

The JSON format includes waypoint names, coordinates, frame IDs, and timestamps.

## Troubleshooting

### "Interactive naming disabled (no TTY)" message

**Solution**: Run the waypoint_collector node in a separate terminal tab (Terminal 2 as shown in Quick Start)

### Waypoints saved to `default_map`

**Solution**: Make sure you launched Terminal 1 with the intended `map_name`; the launch file publishes it on `/map_tools/map_name` and the collector uses that latched value.

### Waypoints not appearing on map

1. Verify the map file exists:
   ```bash
   ls src/map_tools/maps/<map_name>/
   ```

2. Check the RViz configuration is loading:
   ```bash
   ros2 topic list | grep waypoints
   ```

3. Ensure the waypoint_collector node is running:
   ```bash
   ros2 node list | grep waypoint_collector
   ```

4. Ensure RViz has both displays enabled:
   - `Waypoints` (MarkerArray)
   - `Waypoint Editor` (InteractiveMarkers)

### Can't click on map in RViz

1. Select the "Publish Point" tool from the RViz toolbar (top-left area)
2. Ensure the map frame is visible and set as the fixed frame in RViz
3. Check that the waypoint_collector is subscribed to `/clicked_point`

## Adding New Maps

1. Create a new map directory:
   ```bash
   mkdir -p src/map_tools/maps/my_map
   ```

2. Add the map files:
   - `my_map.pgm` - Map image
   - `my_map.yaml` - Map metadata

3. Use the map:
   ```bash
   ros2 launch map_tools waypoint_collector.launch.py map_name:=my_map
   ```


## License

See LICENSE file in repository root.
