# Phase Checklist

## Phase 0: Isolation

- Work stays under `ros2_ws/src/scoutmini_social_perception`.
- Existing dirty files are read-only unless a later phase explicitly requires a wrapper integration.
- `git status --short` should show the new package as the only new project work.

## Phase 1: Fake Pipeline

- Launch fake people with no camera and no ML.
- Verify `/people/tracks` and `/people/markers`.
- This proves ROS packaging, markers, topic schema, and benchmarking.

## Phase 2: YOLO Detection

- Use `/equirectangular/image` bag replay first.
- Do not download weights implicitly from code.
- If `model_path` is empty or missing, node must log a clear error and keep running without crashing.

## Phase 3: Tracking

- Track IDs remain stable across consecutive frames.
- Stale tracks clear quickly.
- Output stays in JSON plus markers until AdaSCoRe format is confirmed.

## Phase 4: Projection

- Convert equirectangular pixel center to bearing.
- Publish base-frame person estimates.
- Range starts with `fixed_distance_debug`; depth/range fusion is separate.

## Phase 5: AdaSCoRe Adapter

- Adapter remains disabled by default.
- Confirmed AdaSCoRe uses `people_msgs/msg/People` on `/people`.
- Keep `people_msgs` output optional so this package still builds without AdaSCoRe dependencies.
- Require people coordinates in the AdaSCoRe controller frame, normally `map`.
- Transform projected people through TF before the AdaSCoRe adapter path.
- Keep debug mirror topics enabled.

## Phase 6: Robot Integration

- Add opt-in wrapper launches only.
- Do not edit current `basic.launch.py`, `navigation.launch.py`, or `nav2.yaml` unless wrappers are proven insufficient.
