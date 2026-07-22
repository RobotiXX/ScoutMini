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

The AdaSCoRe shadow controller is lifecycle-managed and isolated from the
robot. Its only velocity output is `/adascore/shadow/cmd_vel`:

```bash
ros2 launch scoutmini_social_navigation adascore_shadow.launch.py
```

Supervised hardware tests use a separate gate that starts disarmed, clamps
velocity, stops on stale input, and automatically disarms after two seconds:

```bash
ros2 launch scoutmini_social_navigation supervised_motion_gate.launch.py
ros2 service call /adascore/motion_gate/enable std_srvs/srv/SetBool "{data: true}"
```

Do not arm the gate without a clear test area, an operator at the emergency
stop, healthy shadow diagnostics, and a verified isolated ROS domain.

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

The replay scan is reduced to roughly 180 angular samples before AdaSCoRe.
This preserves two-degree obstacle coverage while bounding the social
force planner's per-agent obstacle work. `/adascore/shadow/diagnostics` reports
an error if command output stalls while People input remains fresh.
New fused tracks require two observations, and lower-confidence range
associations within 0.30 m of another person are withheld from AdaSCoRe.

The GMU bags do not provide camera-to-LiDAR calibration. The replay uses the
ScoutMini URDF transform only to exercise the full interface, and the video
labels resulting ranges as unverified scan associations. Do not treat those
ranges as ground truth or use them to tune motion behavior.
