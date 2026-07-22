# scoutmini_sim

Simulation launch and world assets for the Scout Mini robot.

This package owns the Gazebo/ROS simulation entry points. The real robot launch
files live in the robot bringup/nav packages; use this package when you want to
run Gazebo, simulated Nav2, simulated route execution, or SLAM in simulation.

## Build

From the workspace root:

```bash
cd /ros2_ws
colcon build --packages-select scoutmini_sim scoutmini_description scoutmini_nav2 scoutmini_tasks
source install/setup.bash
```

If you changed launch files, worlds, configs, or Python nodes, rebuild and
source the workspace again before testing.

## Launch Gazebo Only

```bash
ros2 launch scoutmini_sim gazebo_mini.launch.py
```

This starts Gazebo, spawns the Scout Mini robot, starts
`robot_state_publisher`, starts the ROS-Gazebo bridge, spawns the Gazebo
controllers, and starts the Scout Mini topic relay.

Useful arguments:

```bash
ros2 launch scoutmini_sim gazebo_mini.launch.py world:=warehouse
ros2 launch scoutmini_sim gazebo_mini.launch.py world:=empty
ros2 launch scoutmini_sim gazebo_mini.launch.py world:=default_warehouse
ros2 launch scoutmini_sim gazebo_mini.launch.py world:=tb3_sandbox
ros2 launch scoutmini_sim gazebo_mini.launch.py world:=fuse_3rd
```

The Fuse 3rd floor world includes `door_1`, a hinged door that is closed at
`0.0` radians and can swing from `-1.5708` to `1.5708` radians. The slider commands exact Gazebo model poses about the hinge, so it does not depend on PID tuning or physics settling:

```bash
ros2 launch scoutmini_sim gazebo_mini.launch.py world:=fuse_3rd door_slider:=true
```

The launch file bridges Gazebo's `/world/fuse_3rd/set_pose` service into ROS, and the slider calls that ROS service directly. It does not require the `gz` command to be on `PATH`.


Spawn pose:

```bash
ros2 launch scoutmini_sim gazebo_mini.launch.py \
  world:=fuse_3rd \
  spawn_x:=0.0 \
  spawn_y:=0.0 \
  spawn_z:=0.05 \
  spawn_yaw:=0.0
```

Use a custom SDF world:

```bash
ros2 launch scoutmini_sim gazebo_mini.launch.py \
  world_file:=/absolute/path/to/world.sdf \
  world_name:=my_world
```

Set `spawn_robot:=false` if you only want to open the world without spawning
the robot.

## Launch Full 2D Navigation Simulation

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py
```

This includes `gazebo_mini.launch.py`, starts the Nav2 stack using
`scoutmini_sim/config/nav2_sim.yaml`, publishes the initial pose in sim mode,
and provides a static `map -> odom` transform when AMCL is not configured to
publish it.

Common arguments:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py world:=fuse_3rd map_name:=fuse_3rd
ros2 launch scoutmini_sim simulation_2d.launch.py rviz:=true
ros2 launch scoutmini_sim simulation_2d.launch.py amcl_tf_broadcast:=true
```

Initial pose:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py \
  initial_pose_x:=0.0 \
  initial_pose_y:=0.0 \
  initial_pose_yaw:=0.0
```

Send one test goal automatically after startup:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py \
  use_goal:=true \
  goal_x:=1.0 \
  goal_y:=0.0 \
  goal_yaw:=0.0
```

## Launch Route Loop Runner

Start the navigation sim first:

```bash
ros2 launch scoutmini_sim simulation_2d.launch.py world:=fuse_3rd map_name:=fuse_3rd
```

Then in another terminal:

```bash
source /ros2_ws/install/setup.bash
ros2 launch scoutmini_sim route_loop_runner.launch.py \
  map_name:=fuse_3rd \
  route_name:=route1
```

The route runner sends goals to Nav2's `/navigate_through_poses` action. Route
files are loaded from:

```text
map_tools/maps/<map_name>/routes/<route_name>.yaml
```

For example:

```text
map_tools/maps/fuse_3rd/routes/route1.yaml
```

Useful arguments:

```bash
ros2 launch scoutmini_sim route_loop_runner.launch.py route_loop:=false
ros2 launch scoutmini_sim route_loop_runner.launch.py route_start_delay_sec:=15.0
ros2 launch scoutmini_sim route_loop_runner.launch.py route_repeat_delay_sec:=2.0
ros2 launch scoutmini_sim route_loop_runner.launch.py route_skip_missing_waypoints:=true
```

By default, this sim launch loops the route. Set `route_loop:=false` to run the
route once.

## Launch SLAM Toolbox For Simulation

```bash
ros2 launch scoutmini_sim slam_toolbox_sim.launch.py
```

This starts `slam_toolbox` with sim time enabled. By default it uses:

```text
scoutmini_nav2/config/mapper_params_online_async.yaml
```

To use a different params file:

```bash
ros2 launch scoutmini_sim slam_toolbox_sim.launch.py \
  params_file:=/absolute/path/to/mapper_params.yaml
```

## Notes

- Source the workspace in every new terminal before launching.
- Use `simulation_2d.launch.py` when you want Nav2 to move the robot.
- Use `gazebo_mini.launch.py` when you only need Gazebo and the simulated robot.
- Use `route_loop_runner.launch.py` separately after Nav2 is running.
- Built-in worlds are `warehouse`, `empty`, `default_warehouse`, `tb3_sandbox`,
  and `fuse_3rd`.
