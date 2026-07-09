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

Observed on this robot:

- `people_msgs`, `hunav_msgs`, `hunav_evaluator`, `hunav_rviz2_panel`,
  `hunav_sim`, `pic4rl`, `hunav_agent_manager`,
  `social_force_window_planner`, and `adascore` built in
  `/home/nvidia/adascore_ws`.
- AdaSCoRe's upstream `.repos` file did not include
  `social_force_window_planner`, but the Nav2 social-window configs reference
  `social_force_window_planner::SFWPlannerNode`. The ScoutMini manifest adds
  `https://github.com/robotics-upo/social_force_window_planner.git` on the
  `ros2` branch.
- Two external dependency checkouts needed local build-compatibility patches on
  this robot:
  `hunav/hunav_sim/hunav_agent_manager` was adjusted from
  `behaviortree_cpp` to `behaviortree_cpp_v3`, from
  `BT::Tree::tickExactlyOnce()` to `BT::Tree::tickRoot()`, and to include
  lightsfm headers from the workspace source tree.
  `social_force_window_planner` was adjusted to include the workspace lightsfm
  headers directly instead of assuming `/usr/local/include/lightsfm`.
- `rosdep` attempted to install additional apt packages and needed sudo, so
  system dependency installation was not completed from Codex.
- AdaSCoRe's Python `SocialForceModel` import and `/people` subscription path
  were validated with `/people` remapped to a dry-run topic.
- `adascore_readiness_check` reports `adascore_dependencies_available: true`
  after sourcing `/home/nvidia/adascore_ws/install/setup.bash` and the
  ScoutMini overlay.
- GPU inference is not complete yet: TensorRT is installed, but the default
  `/home/nvidia/models/yolo/yolo11n.engine` engine does not exist and the
  installed PyTorch build is CPU-only.

## Source Notes

- AdaSCoRe upstream: `https://github.com/maurom3197/adascore`, `humble` branch.
- AdaSCoRe publishes an upstream `adascore.repos` for companion dependencies.
- `wg-perception/people` documents `ros2` as its ROS 2 branch and contains
  `people_msgs`.
- HuNav repositories used `v1.0-humble` branches on this robot, not a plain
  `humble` branch.
