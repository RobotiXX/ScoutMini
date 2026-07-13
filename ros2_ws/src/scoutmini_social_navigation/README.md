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
