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

Readiness checker:

```bash
ros2 run scoutmini_social_perception adascore_readiness_check
```

Result:

- Test suite passed with 9 tests after adding the checker shape test.
- Build passed.
- `/home/nvidia/models/yolo/yolo11n.pt` exists.
- `/home/nvidia/models/yolo/yolo11n.engine` does not exist.
- TensorRT is installed, but `yolo_gpu_execution_ready` is false because CUDA PyTorch is unavailable and no default TensorRT engine artifact exists.
- AdaSCoRe dependencies remain unavailable according to the checker.

AdaSCoRe dry-run launch:

```bash
ros2 launch scoutmini_social_perception adascore_dry_run.launch.py
ros2 topic echo --once --full-length /adascore/dry_run/people
ros2 topic echo --once --full-length /adascore/people_debug
```

Result:

- Dry run launched fake projected people, a static identity TF between isolated dry-run frames, `people_frame_transform`, and the adapter in `json_debug` mode.
- `/adascore/dry_run/people` published JSON with `frame_id: "adascore_dry_map"` and two fake people.
- `/adascore/people_debug` reported `enabled: true`, `output_message_type: "json_debug"`, `frame_id: "adascore_dry_map"`, `adascore_frame_id: "adascore_dry_map"`, `people_count: 2`, and `people_msgs_available: false`.
- The dry run intentionally avoids the live `map`/`base_link` TF tree by default; use `target_frame:=map source_frame:=base_link` only for an intentional live-TF check.

AdaSCoRe message-contract unit gate:

```bash
python3 -m compileall -q src/scoutmini_social_perception/scoutmini_social_perception
colcon test --packages-select scoutmini_social_perception --event-handlers console_direct+
colcon build --symlink-install --packages-select scoutmini_social_perception
```

Result:

- Test suite passed with 11 tests.
- Build passed.
- Added unit coverage for projected-person conversion into the expected `people_msgs/People` contract: header stamp/frame, person name, x/y/yaw, velocity x/y/angular z, reliability, and adapter tags.
- Added unit coverage for the `require_frame_match` gate.

TensorRT model-artifact readiness gate:

```bash
ros2 run scoutmini_social_perception adascore_readiness_check \
  --yolo-engine-path /home/nvidia/models/yolo/yolo11n.engine
```

Result:

- Test suite passed with 11 tests.
- Build passed.
- Readiness checker reports `/home/nvidia/models/yolo/yolo11n.pt` as available with `format: "pytorch"`.
- Readiness checker reports `/home/nvidia/models/yolo/yolo11n.engine` as missing with `format: "tensorrt"`.
- `summary.yolo_gpu_execution_ready` remains false until CUDA PyTorch works or the TensorRT engine artifact exists.

AdaSCoRe dependency manifest gate:

```bash
python3 -m compileall -q src/scoutmini_social_perception/scoutmini_social_perception
colcon test --packages-select scoutmini_social_perception --event-handlers console_direct+
colcon build --symlink-install --packages-select scoutmini_social_perception
```

Result:

- Test suite passed with 12 tests.
- Build passed.
- Added separate `.repos` manifests for `people_msgs`-only import and full upstream AdaSCoRe import.
- Added manifest parsing coverage to confirm the expected People `ros2` branch and AdaSCoRe `humble` branch are pinned.
- Dependency install instructions remain separate-workspace only; no AdaSCoRe source was vendored into ScoutMini.

AdaSCoRe/Nav2 graph preflight gate:

```bash
ros2 run scoutmini_social_perception adascore_preflight_check --settle-sec 1.0
```

Result on the current robot graph:

- Test suite passed with 14 tests.
- Build passed.
- Preflight observed 124 topics.
- Required topics present: `/tf`, `/rko_lio/odometry`, `/scan`, `/map`.
- Required topic missing: `/people`.
- Motion topic present: `/cmd_vel`.
- `summary.safe_to_start_motion` is always false by design; this command is read-only and cannot authorize motion.

Preflight fail-on-missing gate:

```bash
ros2 run scoutmini_social_perception adascore_preflight_check \
  --settle-sec 1.0 \
  --fail-on-missing
```

Result on the current robot graph:

- Test suite passed with 14 tests.
- Build passed.
- Command exited with code 2 because `/people` is missing.
- Required topics present: `/tf`, `/rko_lio/odometry`, `/scan`, `/map`.
- `summary.safe_to_start_motion` remained false.

Preflight topic-type gate:

```bash
ros2 run scoutmini_social_perception adascore_preflight_check \
  --settle-sec 1.0 \
  --fail-on-missing
```

Result on the current robot graph:

- Test suite passed with 16 tests.
- Build passed.
- Command exited with code 2 because `/people` is missing.
- The preflight reported the expected `/people` type as `people_msgs/msg/People`.
- `/cmd_vel` was visible, so `summary.motion_topics_detected` was true.
- `summary.safe_to_start_motion` remained false.

Real `people_msgs` adapter dry run:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/adascore_ws/install/setup.bash
source /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/install/setup.bash
ros2 launch scoutmini_social_perception adascore_dry_run.launch.py \
  output_message_type:=people_msgs \
  adascore_people_topic:=/adascore/dry_run/people_msg
```

Result:

- `people_msgs` built successfully in `/home/nvidia/adascore_ws`.
- `ros2 interface show people_msgs/msg/People` and `people_msgs/msg/Person` matched the adapter contract.
- `/adascore/dry_run/people_msg` published type `people_msgs/msg/People`.
- The message contained two fake people in `frame_id: adascore_dry_map` with x/y/yaw, velocity, reliability, and adapter tags.
- `/adascore/people_debug` reported `output_message_type: "people_msgs"` and `people_msgs_available: true`.
- This remained a no-motion dry run on `/adascore/dry_run/people_msg`, not `/people`.

AdaSCoRe external workspace bring-up:

```bash
mkdir -p /home/nvidia/adascore_ws/src
vcs import /home/nvidia/adascore_ws/src < deps/people_msgs_ros2.repos
colcon build --symlink-install --packages-select people_msgs
vcs import /home/nvidia/adascore_ws/src < deps/adascore_upstream_humble.repos
colcon build --symlink-install --packages-select \
  people_msgs hunav_msgs hunav_evaluator hunav_rviz2_panel hunav_sim pic4rl adascore
```

Result:

- `people_msgs` built and exposed `people_msgs/msg/People` and `people_msgs/msg/Person`.
- `adascore`, `hunav_msgs`, `hunav_evaluator`, `hunav_rviz2_panel`,
  `hunav_sim`, `pic4rl`, `hunav_agent_manager`, and
  `social_force_window_planner` built in `/home/nvidia/adascore_ws`.
- `hunav_agent_manager` required a local external-workspace compatibility patch
  for `behaviortree_cpp_v3`, `BT::Tree::tickRoot()`, and workspace-local
  lightsfm headers.
- `social_force_window_planner` required a local external-workspace
  compatibility patch for workspace-local lightsfm headers.
- `rosdep` also attempted to install apt package `ros-humble-controller-manager` and required sudo credentials, so system dependency installation was not completed.
- `adascore_readiness_check` now finds `adascore`, `people_msgs`,
  `hunav_msgs`, `hunav_agent_manager`, and `social_force_window_planner`, and
  reports `adascore_dependencies_available: true`.
- GPU model readiness is still incomplete: TensorRT is available, but
  `/home/nvidia/models/yolo/yolo11n.engine` is missing and the installed torch
  build reports `cuda_available: false`.

AdaSCoRe SFM read-only consumption smoke:

```bash
ros2 launch scoutmini_social_perception adascore_dry_run.launch.py \
  output_message_type:=people_msgs \
  adascore_people_topic:=/adascore/dry_run/people_msg
```

Then a read-only Python probe instantiated `adascore.utils.sfm.SocialForceModel`
with `/people` remapped to `/adascore/dry_run/people_msg`.

Result:

- `SocialForceModel` imported successfully after building `pic4rl`.
- The probe logged AdaSCoRe's People-topic subscription path.
- The probe received two fake people from the dry-run `people_msgs/msg/People` topic.
- No live `/people` publish was used for this smoke.
- No movement command or Nav2 controller launch was run.

AdaSCoRe dependency readiness after external planner import:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/adascore_ws/install/setup.bash
source /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/install/setup.bash
ros2 run scoutmini_social_perception adascore_readiness_check
```

Result:

- ROS packages available: `adascore`, `people_msgs`, `hunav_msgs`,
  `hunav_agent_manager`, `social_force_window_planner`, `nav2_controller`, and
  `tf2_ros`.
- `summary.adascore_dependencies_available` was `true`.
- `summary.gpu_runtime_detected` and `summary.tensorrt_available` were `true`.
- `summary.yolo_gpu_execution_ready` was `false` because the TensorRT engine is
  not generated yet and torch is CPU-only.
- No movement command or Nav2 controller launch was run.

AdaSCoRe shadow bag validation:

```bash
ros2 launch scoutmini_social_perception bag_adascore_shadow_pipeline.launch.py \
  target_fps:=1.0 \
  imgsz:=416 \
  publish_debug_image:=false
```

Then, while the launch was running:

```bash
ros2 run scoutmini_social_perception perception_bag_validate \
  --duration-sec 28 \
  --people-topic /adascore/shadow/people \
  --min-messages /people/detector_metrics=1 \
  --min-messages /people/detections_2d=1 \
  --min-messages /people/tracks_2d=1 \
  --min-messages /people/projected=1 \
  --min-messages /adascore/shadow/people=1 \
  --require-frame /people/projected=adascore_bag_map \
  --require-frame /adascore/shadow/people=adascore_bag_map \
  --fail-on-missing
```

and:

```bash
ros2 bag play /home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05 \
  --clock \
  --rate 1.0
```

Result:

- Validation passed with no failures.
- `/people/detector_metrics`, `/people/detections_2d`, `/people/tracks_2d`,
  `/people/projected`, and `/adascore/shadow/people` each received 9 messages.
- `/adascore/shadow/people` published `people_msgs/msg/People` with
  `frame_id: adascore_bag_map`, `max_people: 2`, and 7 nonempty messages.
- Detector timing for this CPU run was approximately 116-136 ms per processed
  frame at `imgsz:=416` and `target_fps:=1.0`.
- No live `/people` publish, Nav2 controller launch, or motion command was run.

AdaSCoRe SFM shadow consumption from bag-derived people:

```bash
ros2 run scoutmini_social_perception adascore_sfm_shadow_probe \
  --people-topic /adascore/shadow/people \
  --agents-config social_nav.yaml \
  --duration-sec 30 \
  --min-people 1 \
  --fail-on-missing
```

Then replayed the same short `/equirectangular/image` bag.

Result:

- AdaSCoRe `SocialForceModel` imported and subscribed through the remap from
  `/people` to `/adascore/shadow/people`.
- The stricter probe passed with `messages_received: 1`, `max_people: 1`,
  `last_people_count: 1`, `sfm_last_people_count: 1`, `sfm_max_people: 1`, and
  no error. Passing now requires both the direct shadow topic subscriber and the
  AdaSCoRe `SocialForceModel` state to observe a nonempty people message.
- No live `/people` publish, Nav2 controller launch, or motion command was run.

Longer bag playback check:

```bash
ros2 bag play /home/nvidia/ssd/valicor_bags/rosbag2_2026_06_23-16_18_31 \
  --clock \
  --rate 4.0
```

Result:

- The bag metadata lists `/equirectangular/image`, but direct
  `ros2 topic echo --once /equirectangular/image --field header` subscribers did
  not receive messages during playback with default or `sensor_data` QoS.
- The shadow perception validator also saw no messages from this bag.
- This bag remains a candidate for future playback investigation and is not a
  passing AdaSCoRe shadow-validation gate.

AdaSCoRe social-force controller shadow configure:

```bash
ros2 launch scoutmini_social_perception adascore_shadow_controller.launch.py
```

Then, while the launch was running:

```bash
ros2 lifecycle set /controller_server configure
```

Result:

- `controller_server` launched without lifecycle activation and published no
  live robot command topic because the launch remaps `cmd_vel` to
  `/adascore/shadow/cmd_vel`.
- The lifecycle configure transition succeeded.
- Nav2 created `FollowPath` with type
  `social_force_window_planner::SFWPlannerNode`.
- The social-force sensor interface reported `/scan`,
  `/adascore/shadow/people`, and `/rko_lio/odometry`.
- The launch shut down cleanly after `SIGINT`.
- No activation, Nav2 goal, live `/cmd_vel`, or robot movement was used.
