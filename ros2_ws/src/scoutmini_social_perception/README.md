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
  model_path:=${HOME}/models/yolo/yolo11n_960.engine \
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

## Recorded corpus validation

Build a grouped tune/holdout manifest without replaying motor topics:

```bash
ros2 run scoutmini_social_perception build_tracking_corpus \
  --root /home/nvidia/ssd \
  --output /path/to/corpus_manifest.json \
  --csv /path/to/corpus_manifest.csv \
  --only-insta360
```

Run or resume deterministic tracking windows through the actual ROS detector:

```bash
ros2 run scoutmini_social_perception run_tracking_corpus \
  --manifest /path/to/corpus_manifest.json \
  --output-dir /path/to/results \
  --continue-on-error
```

Long bags use three evenly spaced windows by default; short bags run in full.
Replay defaults to 0.25x so serialized inference can process the requested
sim-time frame rate without silently dropping camera callbacks.
Each result contains intrinsic continuity metrics, suspected-event CSV, a short
review reel, and a blank human-verdict sheet. Suspected events are not counted
as confirmed ID switches until reviewed. Recorded ACME tracks are reported only
as a descriptive baseline, never as human-annotated ground truth.
