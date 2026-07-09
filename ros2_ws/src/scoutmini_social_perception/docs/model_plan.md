# Model Plan

## Source

No YOLO weights were found locally during planning. Ultralytics is installed, TensorRT is installed, and PyTorch appears CPU-only in the current shell.

Default model source is official Ultralytics pretrained COCO detection weights. The initial candidates are:

- `yolo11n.pt`: first benchmark target
- `yolo11s.pt`: fallback if nano accuracy is not acceptable

Weights should be stored outside source control, for example:

`/home/nvidia/models/yolo/yolo11n.pt`

TensorRT engines should also stay outside source control:

`/home/nvidia/models/yolo/yolo11n_fp16.engine`

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
