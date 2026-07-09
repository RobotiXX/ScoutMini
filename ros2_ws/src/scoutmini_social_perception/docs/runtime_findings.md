# Runtime Findings

## Current ML Environment

Observed on 2026-07-08:

- `ultralytics`: `8.4.30`
- `torch`: `2.6.0+cpu`
- `torch.cuda.is_available()`: `False`
- `tensorrt`: `10.3.0`
- `onnx`: not installed
- `onnxruntime`: not installed
- `sahi`: not installed
- `scipy`: not installed

Implications:

- CPU PyTorch is enough for functional bag smoke tests, but not for real-time robot operation.
- TensorRT export is not currently ready from this Python environment because CUDA-enabled PyTorch and ONNX/export dependencies are missing.
- TensorRT is installed, but YOLO GPU execution is not ready until either CUDA PyTorch works or a TensorRT engine exists at the selected model path.
- Ultralytics internal tracking is disabled by default because it attempted to use missing SciPy-dependent tracking code.
- SAHI should remain a documented optional evaluation path until installed and benchmarked.

## Model Artifact

Downloaded outside source control:

`/home/nvidia/models/yolo/yolo11n.pt`

This path is intentionally outside the ScoutMini repo. The package `.gitignore`
also ignores model/export artifacts if someone accidentally stages them inside
the package.

## Next Performance Gate

Before any live robot navigation:

1. Establish a CUDA/TensorRT-capable model path, preferably `.engine`.
2. Run `/people/detector_metrics` on a bag at the intended camera rate.
3. Confirm end-to-end `/people/projected` rate and latency are compatible with Nav2/AdaSCoRe.
4. Keep perception-only launches separate from motion-capable launches.

Repeatable check:

```bash
ros2 run scoutmini_social_perception adascore_readiness_check
```
