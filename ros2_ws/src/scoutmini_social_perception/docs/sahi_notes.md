# SAHI Notes

SAHI, or Slicing Aided Hyper Inference, is useful for high-resolution images
where people are small in the frame. It slices a large image, runs a detector on
each slice, and merges detections back into the original image coordinates.

Relevant sources:

- SAHI docs: https://obss.github.io/sahi/
- Ultralytics SAHI guide: https://docs.ultralytics.com/guides/sahi-tiled-inference/

## Fit for ScoutMini

SAHI is a good candidate for offline evaluation and an optional high-accuracy
mode on `/equirectangular/image`, especially for far-away people in the 360
panorama. It should not be the default real-time mode until measured on the
Jetson because slicing multiplies the number of model invocations per frame.

## Proposed Evaluation

Add a later optional detector mode:

- `inference_mode: full_frame | sahi_sliced`
- `slice_width`, `slice_height`
- `overlap_width_ratio`, `overlap_height_ratio`
- `postprocess_match_threshold`

Suggested first settings for a 1920x640 crop:

- slice size: `640x640`
- width overlap: `0.2`
- height overlap: `0.0`
- max target rate: start at `1 Hz`

Compare against full-frame YOLO on the same bag:

- person recall at distance
- false positives at slice boundaries
- end-to-end latency
- CPU/GPU load
- impact on tracking stability

## Decision

Do not add SAHI to the live pipeline yet. Add it only after the full-frame YOLO
path is stable and benchmarked. If it improves far-person recall enough, expose
it as an opt-in launch/config mode for slow or offline runs before considering
live navigation.
