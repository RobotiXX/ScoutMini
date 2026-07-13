# ScoutMini Social Perception

This package publishes timestamp-preserving, typed person tracks from an
Insta360 equirectangular image. Public IDs are reconciled across the panorama
seam and short BoT-SORT gaps.

## Jetson setup

From this package directory:

```bash
./scripts/install_jetson_dependencies.sh
./scripts/prepare_models.sh
```

Model artifacts stay outside Git under `${HOME}/models/yolo` by default.
`reid_model_path` must be explicit; when it is empty, ReID is disabled instead
of downloading a model during startup.

## Build and run

```bash
cd /path/to/ScoutMini/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scoutmini_social_perception
source install/setup.bash
ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=${HOME}/models/yolo/yolo11n.engine \
  reid_model_path:=${HOME}/models/yolo/yolo26n-cls.pt \
  device:=0
```

Use `input_type:=compressed` and set `image_topic` for ACME merged bags. The
primary output is `vision_msgs/msg/Detection2DArray` on `/people/tracks_2d`;
detector health and inference time publish on `/people/detector_diagnostics`.

The representative GMU test suite is provisioned outside Git with:

```bash
./test/download_acme_gmu_suite.sh /path/on/ssd/acme_gmu_tracking_eval
```
