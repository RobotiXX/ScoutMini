# ScoutMini Social Navigation

This package converts typed panorama tracks into `people_msgs/msg/People` only
when a fresh VLP16 scan supplies a valid range. Points and velocities publish
in `odom` after timestamped TF transforms.

Source the local AdaSCoRe overlay before building and running:

```bash
source /opt/ros/humble/setup.bash
source /path/to/adascore_ws/install/setup.bash
cd /path/to/ScoutMini/ros2_ws
colcon build --symlink-install --packages-select scoutmini_social_navigation
source install/setup.bash
ros2 launch scoutmini_social_navigation people_fusion.launch.py
```

The validated output is `/adascore/shadow/people`. Test the geometry and
message contract without hardware:

```bash
python3 src/scoutmini_social_navigation/test/fusion_smoke.py
```

The AdaSCoRe shadow controller is lifecycle-managed but cannot command the
robot. Its only velocity output is `/adascore/shadow/cmd_vel`:

```bash
ros2 launch scoutmini_social_navigation adascore_shadow.launch.py
```

## Recorded verification video

The GMU analysis runner replays only camera, point-cloud, and odometry topics.
It reconstructs TF from normalized odometry, excludes the bag's recorded
`/cmd_vel` and `/joy`, starts no Scout base node, and runs in an isolated ROS
domain. Source the ScoutMini and local AdaSCoRe overlays, then run:

```bash
$(ros2 pkg prefix scoutmini_social_navigation)/share/scoutmini_social_navigation/scripts/render_gmu_analysis.sh \
  /path/to/innovation_1_1 \
  /path/to/rendered
```

The output directory contains the current-pipeline analysis MP4, a side-by-side
comparison against the bag's recorded tracker, frame-level CSV, summary JSON,
the derived ROS bag, and the replay logs. Recorded tracks are a baseline, not
human-annotated ground truth. The synthetic four-meter path exists only to make
AdaSCoRe publish its candidate trajectories and shadow command.

The GMU bags do not provide camera-to-LiDAR calibration. The replay uses the
ScoutMini URDF transform only to exercise the full interface, and the video
labels resulting ranges as unverified scan associations. Do not treat those
ranges as ground truth or use them to tune motion behavior.
