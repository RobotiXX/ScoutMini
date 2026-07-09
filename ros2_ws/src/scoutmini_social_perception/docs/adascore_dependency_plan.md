# AdaSCoRe Dependency Plan

This package does not vendor AdaSCoRe, HuNavSim, `people_msgs`, or simulation
repositories into ScoutMini. Use a separate workspace so dependency experiments
do not touch unrelated ScoutMini files.

Recommended external workspace:

```bash
mkdir -p /home/nvidia/adascore_ws/src
cd /home/nvidia/adascore_ws
source /opt/ros/humble/setup.bash
```

## Phase 4A: `people_msgs` only

Use this first to unlock the adapter's real `people_msgs/msg/People` output
without importing the full AdaSCoRe simulation stack:

```bash
vcs import src < /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/src/scoutmini_social_perception/deps/people_msgs_ros2.repos
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 interface show people_msgs/msg/People
ros2 interface show people_msgs/msg/Person
```

After this passes, re-source ScoutMini's workspace and run:

```bash
ros2 launch scoutmini_social_perception adascore_adapter_pipeline.launch.py \
  enabled:=true \
  output_message_type:=people_msgs \
  adascore_people_topic:=/people \
  adascore_frame_id:=map
```

Keep robot motion disabled while validating `/people`.

## Phase 4B: Full upstream AdaSCoRe import

Use this only after the People-message adapter path is validated:

```bash
vcs import src < /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/src/scoutmini_social_perception/deps/adascore_upstream_humble.repos
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep -E 'adascore|people_msgs|hunav|social_force'
```

The full manifest includes simulation and Jackal-related upstream dependencies.
Do not merge those packages into the ScoutMini repository. Keep any build fixes
or local patches in `/home/nvidia/adascore_ws` until they are understood and
reviewed.

## Source Notes

- AdaSCoRe upstream: `https://github.com/maurom3197/adascore`, `humble` branch.
- AdaSCoRe publishes an upstream `adascore.repos` for companion dependencies.
- `wg-perception/people` documents `ros2` as its ROS 2 branch and contains
  `people_msgs`.
