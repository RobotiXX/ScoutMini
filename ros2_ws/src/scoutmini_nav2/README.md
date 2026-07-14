# Scout Mini Nav2

This package provides the Nav2 configuration and robot navigation launch for
the Scout Mini. Gazebo simulation launches live in `scoutmini_sim`.

For the full Gazebo + Nav2 simulation flow, use:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py
```

By default, the simulation uses the Fuse 3rd floor world and map:

- world: `fuse_3rd`
- world file: `map_tools/maps/fuse_3rd/fuse_3rd.sdf`
- map: `map_tools/maps/fuse_3rd/fuse_3rd.yaml`
- initial pose: `x=0.0`, `y=0.0`, `yaw=0.0`

## Build

From the ROS workspace inside the Docker container:

```bash
cd /ros2_ws
colcon build --packages-select scoutmini_description scoutmini_sim scoutmini_nav2 scoutmini_tasks map_tools
source install/setup.bash
```

## Launch Nav2 On The Robot

Start the Nav2 stack for the robot:

```bash
ros2 launch scoutmini_nav2 navigation.launch.py
```

Useful overrides:

```bash
ros2 launch scoutmini_nav2 navigation.launch.py \
  map_name:=fuse_3rd \
  rviz:=true
```

`navigation.launch.py` is intentionally focused on the navigation stack. It
does not launch Gazebo or route-loop task runners.

## Launch The Simulation

Start Gazebo, spawn Scout Mini, launch Nav2, and localize on the default sim map:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py
```

Send a single autonomous Nav2 goal at launch:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py \
  use_goal:=true \
  goal_x:=2.0 \
  goal_y:=0.0 \
  goal_yaw:=0.0
```


Launch the Fuse 3rd floor world with its matching map:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py \
  world:=fuse_3rd \
  map_name:=fuse_3rd \
  spawn_x:=13.7 \
  spawn_y:=26.0 \
  spawn_yaw:=-2.02 \
  initial_pose_x:=13.7 \
  initial_pose_y:=26.0 \
  initial_pose_yaw:=-2.02
```


For simulated maps generated in metric map coordinates, keep the Gazebo spawn
pose and Nav2 initial pose matched. `/ground_truth/odom` reports the robot pose
relative to the Gazebo world; AMCL publishes `map -> odom`. If the robot is
spawned at `(0, 0, 0)` but initialized in Nav2 at `(13.7, 26.0, -2.02)`, RViz
will draw scans at a different map pose than the physical pose Gazebo is
raycasting from.

Useful overrides:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py \
  world:=empty \
  map_name:=warehouse \
  spawn_x:=0.0 \
  spawn_y:=0.0 \
  initial_pose_x:=0.0 \
  initial_pose_y:=0.0 \
  initial_pose_yaw:=0.0
```

Available `world` values:

- `default_warehouse`: vendored TurtleBot/Nav2 warehouse world
- `warehouse`: Scout Mini local warehouse world
- `empty`: empty world with sensors
- `tb3_sandbox`: TurtleBot3 sandbox-style world
- `fuse_3rd`: STL-based world from `map_tools/maps/fuse_3rd`


## Ground-Truth Localization In Simulation

The simulation launch defaults to ground-truth localization:

```bash
amcl_tf_broadcast:=false
```

With this setting, AMCL still starts as part of Nav2 bringup, but it does not
publish `map -> odom`. The launch publishes a static identity `map -> odom`
instead, so `/ground_truth/odom` remains the source of robot motion in RViz and
Nav2.

This avoids apparent drift when the simulated 3D world and the 2D occupancy map
are not a perfect match. Gazebo raycasts `/scan` against the STL/world geometry,
while AMCL localizes against the PGM occupancy map. If AMCL is allowed to publish
`map -> odom`, it can slowly move the robot pose in RViz even though
`/ground_truth/odom` is still correct relative to Gazebo.

To test AMCL scan matching instead of ground-truth localization, launch with:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py amcl_tf_broadcast:=true
```

When `amcl_tf_broadcast:=true`, do not also publish a static `map -> odom` from
another terminal, or TF will have two publishers for the same transform.

## RViz

The simulation launch forwards `rviz` and `rviz_config_file` arguments to
`navigation.launch.py`. If RViz is launched manually, set:

- Fixed Frame: `map`
- Map display topic: `/map`
- Map display reliability: `Reliable`
- Map display durability: `Transient Local`

Nav2's map server publishes `/map` with transient local durability. If costmaps
show but the normal map does not, the RViz Map display QoS is usually the issue.

Check the map topic from the terminal:

```bash
ros2 topic info /map -v
ros2 topic echo /map --qos-durability transient_local --qos-reliability reliable --once
```

## Nav2 Health Checks

The goal runner waits for `/navigate_to_pose`. If a launch starts but the robot
does not navigate, check that Nav2 lifecycle nodes are active:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
```

Check command output:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /diff_drive_controller/cmd_vel
```

The Scout Mini topic relay converts Nav2 `Twist` commands into stamped commands
for Humble's `diff_drive_controller`.
