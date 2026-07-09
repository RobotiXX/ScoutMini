# Model Plan

## Source

No YOLO weights were found locally during planning. Ultralytics is installed, TensorRT is installed, and PyTorch appears CPU-only in the current shell.

Default model source is official Ultralytics pretrained COCO detection weights. The initial candidates are:

- `yolo11n.pt`: first benchmark target
- `yolo11s.pt`: fallback if nano accuracy is not acceptable

Weights should be stored outside source control, for example:

`/home/nvidia/models/yolo/yolo11n.pt`

TensorRT engines should also stay outside source control:

`/home/nvidia/models/yolo/yolo11n.engine`

## Selection Process

1. Run bag replay against `yolo11n.pt` on `/equirectangular/image`.
2. Export `yolo11n.pt` to TensorRT FP16 and benchmark again.
3. Try `yolo11s.pt` only if the nano model misses too many people.
4. Lock the smallest model that meets latency, stability, and detection quality gates.

Ultralytics internal tracking is optional and disabled by default. The package
uses `people_tracker` for first-pass ID assignment so YOLO detection can run
without extra tracker dependencies such as SciPy. Enable Ultralytics tracking
only after installing and validating those dependencies.

## Real-Time Gates

Before live robot navigation:

- detector processing rate is at least the configured `target_fps`
- end-to-end image-to-projected-person latency is measured and bounded
- stale people disappear within `track_timeout_sec`
- no perception-only launch commands robot motion
- CPU/GPU load leaves room for Nav2 and camera processing

## TensorRT Artifact Gate

After exporting a TensorRT engine outside the repo, verify that the expected
artifact exists and is recognized:

```bash
ros2 run scoutmini_social_perception adascore_readiness_check \
  --yolo-pt-path /home/nvidia/models/yolo/yolo11n.pt \
  --yolo-engine-path /home/nvidia/models/yolo/yolo11n.engine
```

Pass criteria:

- `model_artifacts.yolo_tensorrt_engine.available` is `true`.
- `model_artifacts.yolo_tensorrt_engine.format` is `tensorrt`.
- `summary.yolo_gpu_execution_ready` is `true`.

Then run the same perception launch against the engine:

```bash
ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=/home/nvidia/models/yolo/yolo11n.engine \
  device:=0 \
  target_fps:=8.0 \
  imgsz:=640 \
  publish_debug_image:=false
```

Use `/people/detector_metrics` to compare `elapsed_ms`, observed topic rate,
and dropped-frame behavior against the CPU `.pt` run.
