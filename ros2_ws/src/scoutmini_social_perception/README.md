# ScoutMini Social Perception

This package isolates people detection, tracking, projection, benchmarking, and future AdaSCoRe adapter work for ScoutMini.

The package is intentionally standalone. Existing ScoutMini bringup, Nav2, route, Slack, and camera-driver files should remain untouched until a late opt-in integration phase.

Large runtime artifacts do not belong in this package. Keep model weights,
TensorRT engines, bags, SAHI experiments, and AdaSCoRe checkouts outside this
package unless there is a deliberate later decision to vendor something.

## First Smoke Test

```bash
cd /home/nvidia/repos/ScoutMini/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scoutmini_social_perception
source install/setup.bash
ros2 launch scoutmini_social_perception fake_people_pipeline.launch.py
```

Expected topics:

- `/people/tracks`
- `/people/markers`
- `/perception/benchmark`

## YOLO Bag Test

Provide a local model path first. The code does not implicitly download weights.

```bash
ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py model_path:=/home/nvidia/models/yolo/yolo11n.pt
ros2 bag play /home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05
```

The launch defaults to YOLO detection plus the package's own tracker. Ultralytics
internal tracking can be enabled later, but it may require additional runtime
dependencies.

See `docs/model_plan.md` before choosing or exporting a model.

## AdaSCoRe Boundary

The AdaSCoRe adapter is disabled by default. The inspected AdaSCoRe `humble`
branch expects `people_msgs/msg/People` on `/people`, but `people_msgs` is not
installed in the current robot environment. Keep the adapter in `json_debug`
mode until AdaSCoRe's dependency stack is installed deliberately. The adapter
launch includes a TF-backed `people_frame_transform` node so AdaSCoRe receives
people in `map`, not base-link-relative projection coordinates.

See `docs/adascore_interface_notes.md` before enabling `people_msgs` output.
