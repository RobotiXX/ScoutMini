# AdaSCoRe Shadow Bag Validation

This flow validates the ScoutMini perception-to-AdaSCoRe data path without
publishing to live `/people`, starting Nav2, or commanding robot motion.

## Launch Shadow Pipeline

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/adascore_ws/install/setup.bash
source /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/install/setup.bash
ros2 launch scoutmini_social_perception bag_adascore_shadow_pipeline.launch.py \
  target_fps:=1.0 \
  imgsz:=416 \
  publish_debug_image:=false
```

The launch publishes AdaSCoRe-compatible `people_msgs/msg/People` to
`/adascore/shadow/people`. It intentionally does not publish live `/people`.

## Validate Perception Outputs

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/adascore_ws/install/setup.bash
source /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/install/setup.bash
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

Terminal 3, while the validator is running:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05 \
  --clock \
  --rate 1.0
```

Expected result:

- `/people/detector_metrics`, `/people/detections_2d`, `/people/tracks_2d`,
  `/people/projected`, and `/adascore/shadow/people` all receive messages.
- `/adascore/shadow/people` has type `people_msgs/msg/People`.
- `/people/projected` and `/adascore/shadow/people` use frame
  `adascore_bag_map`.
- Nonempty people messages are observed before the command exits.

## Validate AdaSCoRe SFM Consumption

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/adascore_ws/install/setup.bash
source /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/install/setup.bash
ros2 run scoutmini_social_perception adascore_sfm_shadow_probe \
  --people-topic /adascore/shadow/people \
  --agents-config social_nav.yaml \
  --duration-sec 30 \
  --min-people 1 \
  --fail-on-missing
```

Terminal 3, while the probe is running:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05 \
  --clock \
  --rate 1.0
```

Expected result:

- AdaSCoRe `SocialForceModel` imports successfully.
- Its hard-coded `/people` subscription is remapped to
  `/adascore/shadow/people`.
- The probe receives at least one bag-derived person directly from
  `/adascore/shadow/people`.
- `SocialForceModel` also stores at least one bag-derived person after the
  remap.
- No live `/people`, Nav2 controller, or motion command is used.

## Current Bag Findings

- `/home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05` is the primary
  passing shadow-validation bag. It publishes `/equirectangular/image` and
  produced nonempty people detections on this robot.
- `/home/nvidia/ssd/valicor_bags/rosbag2_2026_06_23-16_18_31` lists
  `/equirectangular/image` in metadata, but direct `ros2 topic echo --once
  /equirectangular/image` subscribers did not receive messages during playback
  with default or `sensor_data` QoS. Treat it as a candidate bag needing further
  playback investigation, not as a passing validation gate.

## Shadow Controller Readiness

`adascore_shadow_controller.launch.py` starts only Nav2's controller server with
`adascore_shadow_controller.yaml`. The config selects
`social_force_window_planner::SFWPlannerNode`, uses `/adascore/shadow/people`,
`/scan`, and `/rko_lio/odometry`, and the launch remaps controller output from
`cmd_vel` to `/adascore/shadow/cmd_vel`.

For no-motion validation, start the launch and only configure the lifecycle node:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/adascore_ws/install/setup.bash
source /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/install/setup.bash
ros2 launch scoutmini_social_perception adascore_shadow_controller.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/adascore_ws/install/setup.bash
source /home/nvidia/repos/ScoutMini-rohan-work/ros2_ws/install/setup.bash
ros2 lifecycle set /controller_server configure
```

Do not activate the controller, send Nav2 goals, or remap this launch back to
live `/cmd_vel` until a supervised movement test is explicitly approved.
