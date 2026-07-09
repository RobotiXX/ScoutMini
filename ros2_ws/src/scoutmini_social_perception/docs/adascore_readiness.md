# AdaSCoRe Readiness Checklist

This checklist defines the gates before ScoutMini perception is allowed to feed AdaSCoRe-controlled robot motion.

## Phase 1: Perception Tuning

Goal: choose a stable person-detection setting before integrating social navigation.

Implementation status:

- Launch exposes `target_fps`, `imgsz`, `confidence_threshold`, `iou_threshold`, and `publish_debug_image`.
- `/people/detector_metrics` records the active values for those tuning parameters.
- Bag smoke has verified that the CPU pipeline publishes `/people/detector_metrics`, `/people/detections_2d`, `/people/tracks_2d`, and `/people/projected`.

Test matrix:

```bash
ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=/home/nvidia/models/yolo/yolo11n.pt \
  target_fps:=1.0 \
  imgsz:=640

ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=/home/nvidia/models/yolo/yolo11n.pt \
  target_fps:=2.0 \
  imgsz:=512

ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=/home/nvidia/models/yolo/yolo11n.pt \
  target_fps:=3.0 \
  imgsz:=416
```

Watch:

```bash
ros2 topic echo /people/detector_metrics
ros2 topic hz /people/detections_2d
ros2 run rqt_image_view rqt_image_view /people/debug_image
```

Pass criteria:

- `/people/detections_2d`, `/people/tracks_2d`, and `/people/projected` publish consistently.
- Debug image boxes match visible people.
- Best CPU setting is documented.
- The selected setting is recorded with average `elapsed_ms`, observed topic rate, and whether debug image publishing was enabled.

## Phase 2: Bearing, Frame, and Range Validation

Goal: prove person positions are in the right direction and frame.

Implementation status:

- Bearing wrap behavior is unit-tested at the equirectangular image center and edges.
- Frame transform math is unit-tested for position and velocity rotation/translation.
- Range-estimation math is unit-tested for fixed debug range and person-height range mode.
- AdaSCoRe yaw selection is unit-tested for explicit yaw, velocity-derived yaw, and bearing fallback.

Checks:

```bash
ros2 topic echo /people/projected
ros2 topic echo /people/projected_map
ros2 run tf2_ros tf2_echo map base_link
```

Pass criteria:

- `bearing_rad` is plausible for front, left, right, and back positions.
- `/people/projected_map` publishes with `frame_id: "map"`.
- Range source is understood. `fixed_distance_debug` is not acceptable for final motion.
- Live walk-around results are recorded in `verification_log.md`.

## Phase 3: GPU and TensorRT Readiness

Goal: move from functional CPU perception to robot-realistic perception speed.

Current known state:

- PyTorch is CPU-only.
- TensorRT is installed.
- ONNX/export dependencies are not yet confirmed.
- `device:=0` is not expected to work until CUDA-enabled PyTorch or a TensorRT engine path is installed.

Checks:

```bash
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.version.cuda)"
python3 -c "import tensorrt; print(tensorrt.__version__)"
```

Targets:

- Short term: CUDA-enabled PyTorch can run `device:=0`.
- Preferred deployment: TensorRT `.engine` model outside the repo, for example `/home/nvidia/models/yolo/yolo11n.engine`.
- Target perception rate before social-navigation motion: 8-15 FPS if the platform supports it.

GPU implementation gate:

1. Install/confirm Jetson-compatible CUDA PyTorch and export dependencies outside this package.
2. Export YOLO to TensorRT using a model artifact path under `/home/nvidia/models/yolo`.
3. Add a separate launch/config setting for the engine path, keeping CPU `.pt` launch behavior intact.
4. Re-run the Phase 1 matrix against the engine and compare `/people/detector_metrics`.

## Phase 4: AdaSCoRe Dependencies

Goal: install the message and controller stack deliberately outside this package.

Required packages:

- `people_msgs` from `wg-perception/people` ROS 2 branch.
- AdaSCoRe `humble` branch.
- HuNavSim dependencies.
- Social force window planner dependencies used by AdaSCoRe Nav2 configs.

Checks:

```bash
ros2 interface show people_msgs/msg/People
ros2 interface show people_msgs/msg/Person
ros2 pkg list | grep -E 'adascore|people_msgs|hunav|social_force'
```

## Phase 5: Adapter Without Motion

Goal: publish valid AdaSCoRe people messages while keeping robot motion disabled.

Launch:

```bash
ros2 launch scoutmini_social_perception adascore_adapter_pipeline.launch.py \
  enabled:=true \
  output_message_type:=people_msgs \
  adascore_people_topic:=/people \
  adascore_frame_id:=map
```

Pass criteria:

- `/people` publishes `people_msgs/msg/People`.
- `header.frame_id` is `map`.
- Person `position.x`, `position.y`, and `position.z` represent x, y, yaw.
- Adapter refuses non-map-frame output when `require_frame_match` is true.

## Phase 6: Nav2/AdaSCoRe Dry Integration

Goal: prove AdaSCoRe starts and consumes the people topic without commanding unsafe motion.

Rules:

- Use a separate opt-in launch/config path.
- Do not replace the default ScoutMini Nav2 launch until dry tests pass.
- Start with social weights disabled or minimal.
- Verify topics before connecting behavior to robot motion.

Required live topics:

```text
/people
/tf
/rko_lio/odometry
/scan
/map
```

## Phase 7: Controlled Motion Gate

Only run after all prior gates pass.

Requirements:

- Open area.
- E-stop ready.
- One simple person/goal scenario.
- Record `/people`, `/people/projected_map`, `/cmd_vel`, `/tf`, `/rko_lio/odometry`, and Nav2 controller logs.
