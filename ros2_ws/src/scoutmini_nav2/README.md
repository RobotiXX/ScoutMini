# Scout Mini Nav2 Simulation

This package provides a Gazebo + Nav2 simulation launch for the Scout Mini.

By default, the simulation uses the vendored TurtleBot/Nav2 warehouse world:

- world: `default_warehouse`
- world file: `scoutmini_description/worlds/turtlebot4_warehouse.sdf`
- map: `map_tools/maps/warehouse/warehouse.yaml`
- initial pose: `x=0.0`, `y=0.0`, `yaw=0.0`

The world SDF is included in this repository, so the launch does not require
`nav2_minimal_tb4_sim` or `turtlebot4_gz_bringup`. The world references Gazebo
Fuel model URLs, so Gazebo needs internet access the first time it loads the
warehouse assets unless those assets are already cached.

## Build

From the ROS workspace inside the Docker container:

```bash
cd /ros2_ws
colcon build --packages-select scoutmini_description scoutmini_nav2 scoutmini_tasks map_tools
source install/setup.bash
```

## Launch The Simulation

Start Gazebo, spawn Scout Mini, launch Nav2, and localize on the warehouse map:

```bash
ros2 launch scoutmini_nav2 simulation_2d.launch.py
```

Send a single autonomous Nav2 goal at launch:

```bash
ros2 launch scoutmini_nav2 simulation_2d.launch.py \
  use_goal:=true \
  goal_x:=2.0 \
  goal_y:=0.0 \
  goal_yaw:=0.0
```

Useful overrides:

```bash
ros2 launch scoutmini_nav2 simulation_2d.launch.py \
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
