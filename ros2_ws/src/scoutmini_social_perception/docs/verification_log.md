# Verification Log

## 2026-07-08

Environment:

- ROS 2 Humble
- Ultralytics `8.4.30`
- TensorRT `10.3.0`
- PyTorch reported as CPU-only from the current shell
- Model downloaded outside the repo: `/home/nvidia/models/yolo/yolo11n.pt`

Commands:

```bash
cd /home/nvidia/repos/ScoutMini/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scoutmini_social_perception
```

Result:

- Build passed.
- Rebuilt after adding the AdaSCoRe optional `people_msgs` adapter path; build still passed.
- `colcon test --packages-select scoutmini_social_perception --event-handlers console_direct+` passed with 4 tests.

Fake pipeline:

```bash
source install/setup.bash
timeout 5s ros2 launch scoutmini_social_perception fake_people_pipeline.launch.py
```

Result:

- `/people/tracks` published at 10 Hz.
- Timeout exit code is expected because the launch was intentionally stopped.

YOLO bag smoke:

```bash
source install/setup.bash
ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=/home/nvidia/models/yolo/yolo11n.pt \
  target_fps:=1.0

ros2 bag play /home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05 \
  --rate 0.25 \
  --topics /equirectangular/image
```

Result:

- Model loaded successfully.
- `/people/detections_2d`, `/people/tracks_2d`, and `/people/projected` all published at the configured 1 Hz during bag playback.
- After adding detector metrics, `/people/detector_metrics`, `/people/detections_2d`, `/people/tracks_2d`, and `/people/projected` all published at the configured 1 Hz during bag playback.
- The first attempt exposed that Ultralytics internal tracking needed SciPy; default tracking was moved to this package's `people_tracker` instead.
- The second attempt exposed and fixed a marker crash when YOLO detections had no incoming `track_id`.

Notes:

- DDS socket warnings can appear in sandboxed commands. Bag-to-node communication required normal ROS graph access.
- CPU-only YOLO is not a real-time target. TensorRT export remains required before live robot performance claims.
- The package is intentionally small and self-contained. Model weights live outside the repo at `/home/nvidia/models/yolo`; no model files, generated engines, SAHI dependency, AdaSCoRe checkout, or bag data are stored in source control.

AdaSCoRe adapter smoke:

```bash
source install/setup.bash
timeout 8s ros2 launch scoutmini_social_perception adascore_adapter_pipeline.launch.py
timeout 8s ros2 launch scoutmini_social_perception adascore_adapter_pipeline.launch.py \
  output_message_type:=people_msgs \
  enabled:=true
```

Result:

- Default debug-only adapter mode started.
- `output_message_type:=people_msgs` started and logged the expected missing `people_msgs` dependency error instead of breaking package import/build.
- Timeout exit code is expected because each smoke launch was intentionally stopped.

AdaSCoRe TF transform smoke:

```bash
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 1 2 0 0 0 0 map base_link
ros2 launch scoutmini_social_perception adascore_adapter_pipeline.launch.py enabled:=false
```

Then one synthetic `/people/projected` JSON message in `base_link` was published.

Result:

- `/people/projected_map` published with `frame_id: "map"`.
- The smoke wrapper timed out during process cleanup, but the expected transformed topic output was captured.

## 2026-07-09

Commands:

```bash
cd /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m compileall -q src/scoutmini_social_perception/scoutmini_social_perception
colcon build --symlink-install --packages-select scoutmini_social_perception
colcon test --packages-select scoutmini_social_perception --event-handlers console_direct+
```

Result:

- Build passed.
- Test suite passed with 5 tests.

YOLO bag smoke:

```bash
ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=/home/nvidia/models/yolo/yolo11n.pt \
  target_fps:=2.0 \
  imgsz:=512 \
  publish_debug_image:=false

ros2 bag play /home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05 \
  --rate 0.25 \
  --topics /equirectangular/image
```

Result:

- `/people/detector_metrics` published with `device: "cpu"`, `target_fps: 2.0`, `imgsz: 512`, `confidence_threshold: 0.35`, `iou_threshold: 0.45`, and `publish_debug_image: false`.
- The benchmark node observed `/people/detector_metrics`, `/people/detections_2d`, `/people/tracks_2d`, and `/people/projected` publishing during bag playback.
- ROS 2 bag and topic commands must run outside Codex's network sandbox; sandboxed runs can report DDS socket errors or hang waiting for topic traffic.

AdaSCoRe dependency gate:

```bash
ros2 interface show people_msgs/msg/People
ros2 pkg list | rg 'adascore|people_msgs|hunav|social_force|nav2_controller|tf2_ros'
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.version.cuda)"
python3 -c "import tensorrt; print(tensorrt.__version__)"
```

Result:

- `people_msgs` is not installed.
- AdaSCoRe, HuNavSim, and social-force planner packages are not installed.
- `nav2_controller`, `tf2_ros`, and `tf2_ros_py` are installed.
- PyTorch is still CPU-only: `2.6.0+cpu`, CUDA unavailable.
- TensorRT Python import works: `10.3.0`.

Adapter missing-dependency smoke:

```bash
timeout 8s ros2 launch scoutmini_social_perception adascore_adapter_pipeline.launch.py \
  enabled:=true \
  output_message_type:=people_msgs \
  adascore_frame_id:=map
```

Result:

- The adapter logged the expected error that `people_msgs` is missing.
- This is the correct fail-closed behavior before Phase 5 can publish real AdaSCoRe people messages.

Phase 2 and adapter math unit gate:

```bash
python3 -m compileall -q src/scoutmini_social_perception/scoutmini_social_perception
colcon test --packages-select scoutmini_social_perception --event-handlers console_direct+
colcon build --symlink-install --packages-select scoutmini_social_perception
```

Result:

- Test suite passed with 8 tests.
- Build passed.
- Added coverage for fixed-range clamping, person-height range estimation, and AdaSCoRe yaw selection from explicit yaw, velocity, or bearing.
