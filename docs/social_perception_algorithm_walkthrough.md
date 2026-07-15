# Social Perception Algorithm Walkthrough

This document specifies the implemented Insta360 person detection, identity
tracking, LiDAR association, odometry-frame velocity estimation, and offline
evaluation pipeline. It distinguishes library behavior from repository-owned
logic and records the equations and defaults needed to reproduce a run.

## 1. System boundary

The runtime data flow is:

```text
Insta360 equirectangular image
  -> YOLO11n person detections
  -> Ultralytics BoT-SORT upstream track IDs
  -> panorama-aware public ID reconciliation
  -> vision_msgs/Detection2DArray on /people/tracks_2d
  -> angular association with a synchronized LaserScan
  -> timestamped scan-to-odom transform
  -> people_msgs/People on /adascore/shadow/people
```

The detector and public-ID logic are implemented in
[`scoutmini_social_perception`](../ros2_ws/src/scoutmini_social_perception).
The scan association and motion estimation are implemented in
[`scoutmini_social_navigation`](../ros2_ws/src/scoutmini_social_navigation).

The pipeline does not estimate monocular depth. A 2D detection is omitted from
`People` unless a fresh LiDAR scan provides an acceptable range and the track
passes the motion and duplicate filters described below.

### Coordinate and symbol conventions

| Symbol | Meaning |
|---|---|
| `W`, `H` | Source panorama width and height in pixels |
| `(x_c, y_c)` | Detection bounding-box center in pixels |
| `w_b`, `h_b` | Detection bounding-box width and height in pixels |
| `theta_c` | Horizontal person bearing in the camera frame |
| `theta_s` | Horizontal person bearing in the scan frame |
| `r` | LiDAR range associated with a detection |
| `p_s` | Person point in the scan frame |
| `p_o` | Person point in the output frame, normally `odom` |
| `dt` | Time between consecutive accepted observations of one public ID |

Angles are radians. ROS transforms are requested at the detection timestamp.
OpenCV colors are stored in BGR order, not RGB order.

## 2. Detector execution and model preparation

### 2.1 Reproducible software and weights

The Jetson installer pins the primary Python inference dependencies:

| Dependency | Version |
|---|---:|
| PyTorch | 2.8.0 Jetson aarch64 wheel |
| torchvision | 0.23.0 Jetson aarch64 wheel |
| Ultralytics | 8.4.30 |
| NumPy | 1.26.4 |
| SciPy | 1.11.4 |
| LAP | 0.5.12 |
| ONNX | 1.17.0 |

`scripts/prepare_models.sh` downloads and verifies these source weights:

| Artifact | SHA-256 |
|---|---|
| `yolo11n.pt` | `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1` |
| `yolo26n-cls.pt` | `0dd6f8dbc448870ac98a3cbb7156f923f7ce21fed3755d4019169ffffd279e81` |

The detection weights are exported to a static `960 x 960` ONNX graph with
opset 17, then built as an FP16 TensorRT engine with a 1024 MiB workspace and
builder optimization level 3. TensorRT engines are hardware- and
TensorRT-version-specific; reproduce them from the verified PyTorch weight
rather than copying an engine between unlike machines.

The node performs one zero-image prediction at startup. This initializes the
backend before a live frame can enter the timed path.

### 2.2 Frame scheduling

The node processes at most `target_fps = 8.0` frames per ROS-clock second. A
frame is skipped when

```math
t_{now} - t_{last} < \frac{1}{f_{target}}.
```

With `use_sim_time=true`, this is replay-clock time. Corpus replay defaults to
`0.25x`, allowing serialized inference to sustain the requested simulated
frame rate without silently accumulating camera callbacks.

### 2.3 YOLO call and output filtering

The node calls `Ultralytics.YOLO.track()` with:

| Argument | Default |
|---|---:|
| Model | `yolo11n_960.engine` |
| Input size | 960 |
| Device | CUDA device `0` on Jetson runs |
| Class filter | COCO class `0`, person only |
| Detection confidence | 0.35 |
| Detection/NMS IoU threshold | 0.45 |
| Persistent tracker state | enabled |
| Tracker | generated runtime copy of `botsort.yaml` |

Image decoding, resize/letterboxing, YOLO head decoding, and non-maximum
suppression are owned by the pinned Ultralytics version. This repository does
not reimplement them. The repository converts every retained box
`(x_1, y_1, x_2, y_2)` to the ROS representation:

```math
x_c = \frac{x_1 + x_2}{2}, \qquad
y_c = \frac{y_1 + y_2}{2}
```

```math
w_b = \max(0, x_2-x_1), \qquad
h_b = \max(0, y_2-y_1).
```

The source timestamp is preserved. The configured frame override defaults to
`360_link`.

## 3. BoT-SORT upstream tracking

BoT-SORT is the first identity layer. It combines motion-based association,
global motion compensation, and optional appearance-based re-identification
(ReID). The exact association implementation is supplied by Ultralytics
8.4.30; this repository fixes its configuration:

| BoT-SORT setting | Value | Function |
|---|---:|---|
| `track_high_thresh` | 0.30 | High-confidence association threshold |
| `track_low_thresh` | 0.10 | Lower boundary for second-stage candidates |
| `new_track_thresh` | 0.35 | Minimum confidence for creating a track |
| `track_buffer` | 24 frames | Retention window for unmatched tracks |
| `match_thresh` | 0.80 | Tracker association cost threshold |
| `fuse_score` | true | Fuse detection score into association cost |
| `gmc_method` | `sparseOptFlow` | Sparse optical-flow global motion compensation |
| `proximity_thresh` | 0.50 | Spatial gate before appearance association |
| `appearance_thresh` | 0.80 | Appearance-distance gate |
| `with_reid` | true | Enable appearance embeddings |
| `model` | explicit path | `yolo26n-cls.pt` in the validated setup |

At 8 processed frames/s, `track_buffer = 24` corresponds nominally to three
seconds of processed frames. Actual retention follows Ultralytics tracker
semantics and the processed frame sequence, not wall-clock time.

The ReID path must be explicit. If it is empty, the node disables ReID and sets
the tracker model to `auto`; it does not download an unrecorded model during
startup. A BoT-SORT ID is treated as an upstream hint, not as the published
identity, because panorama seams and brief upstream fragmentation can still
change it.

## 4. Panorama-aware public ID reconciliation

The second identity layer is repository-owned and fully specified in
[`track_schema.py`](../ros2_ws/src/scoutmini_social_perception/scoutmini_social_perception/track_schema.py).
It maps upstream IDs to monotonic public IDs such as `person_000017`.

### 4.1 Circular image distance

An equirectangular panorama wraps horizontally. For two horizontal centers
`x_1` and `x_2`, the reconciler computes

```math
d_x^{px} = \min\left(
  |x_1-x_2| \bmod W,
  W-(|x_1-x_2| \bmod W)
\right).
```

The normalized 2D center distance is

```math
d = \sqrt{
  \left(\frac{d_x^{px}}{W}\right)^2 +
  \left(\frac{|y_1-y_2|}{H}\right)^2
}.
```

`W` and `H` are the maximum positive dimensions reported by the current and
stored observation. The default nearest-track threshold is `tau = 0.12`.

### 4.2 Assignment procedure

For each frame, observations are processed in detector order. Each public ID
can be assigned at most once in that frame.

```text
expire states older than 1.0 s
for observation in detector order:
    if upstream ID has a live public mapping,
       public ID is unused in this frame,
       and center distance <= 2*tau:
        reuse mapped public ID
    else if the nearest unused live public ID has center distance < tau:
        reuse nearest public ID
    else:
        allocate the next person_NNNNNN ID
    update the public track state and upstream mapping
```

The relaxed `2*tau = 0.24` gate allows a valid upstream identity to move
farther than an appearance-free nearest-neighbor reassociation. The stricter
`tau = 0.12` gate is used when repairing a changed or missing upstream ID.

States expire when

```math
t_{current} - t_{last\ seen} > 1.0\ \text{s}.
```

If timestamps move backward, all active mappings are reset, while the public
ID counter remains monotonic. The reconciler uses only center position and the
upstream ID. It does not use box size, velocity prediction, or ReID embeddings;
those capabilities remain in the upstream BoT-SORT layer. This distinction is
important when reproducing behavior in crowds.

### 4.3 Published 2D contract

`/people/tracks_2d` is a `vision_msgs/msg/Detection2DArray`. Every detection
contains:

- the source image timestamp and `360_link` frame;
- the public ID in `Detection2D.id`;
- center/size bounding-box geometry in source-image pixels;
- class ID `person` and the detector confidence.

An empty but healthy frame publishes an empty array. Inference failure produces
an error diagnostic and no synthetic detections.

## 5. Equirectangular bearing and LiDAR range association

The fusion node consumes a public 2D track and the newest `LaserScan`. It
rejects all detections if the scan is missing or, when both timestamps are
valid,

```math
|t_{track} - t_{scan}| > 0.25\ \text{s}.
```

### 5.1 Pixel-to-bearing mapping

For horizontal center `x_c`, panorama width `W`, normalized forward center
`c_0 = 0.5`, and calibration offset `beta = 0`, the camera-frame bearing is

```math
\theta_c = \operatorname{wrap}_{[-\pi,\pi]}
\left[2\pi\left(\frac{x_c}{W}-c_0\right)+\beta\right].
```

Thus the center pixel points forward, and the full image spans `2*pi`. The
unit horizontal ray in the camera frame is

```math
u_c = [\cos(\theta_c),\ \sin(\theta_c),\ 0]^T.
```

The timestamped camera-to-scan quaternion rotates this direction into the scan
frame:

```math
u_s = R(q_{s<-c})u_c, \qquad
\theta_s = \operatorname{atan2}(u_{s,y},u_{s,x}).
```

Only rotation is applied to this ray. Camera-to-LiDAR translation is not used
when selecting the scan angle. After range selection, the point is constructed
from the LiDAR origin. This is an angular-association approximation and should
not be interpreted as exact cross-sensor ray intersection when the sensor
baseline is significant.

### 5.2 Adaptive angular window

The full angular width represented by a detection box is

```math
\omega_b = 2\pi\frac{w_b}{W}.
```

The scan association uses half-width

```math
h = \operatorname{clamp}(0.30\,\omega_b,\ 0.025,\ 0.18).
```

A scan return `r_i` at angle

```math
\theta_i = \theta_{min} + i\,\Delta\theta
```

is eligible when all conditions hold:

```math
|\operatorname{wrap}(\theta_i-\theta_s)| \le h,
```

```math
r_i \text{ is finite}, \qquad
\max(r_{scan,min},0.35) \le r_i \le \min(r_{scan,max},10.0).
```

At least two eligible returns are required.

### 5.3 Foreground-biased robust range

Eligible returns are sorted in ascending range. For `N` values, define

```math
k = \max(2,\lceil0.25N\rceil).
```

The associated range is

```math
r = \operatorname{median}\left(
  \operatorname{sort}(r_1,\ldots,r_N)_{1:k}
\right).
```

Using the nearest quartile biases the estimate toward the foreground object
inside a box while the median suppresses isolated minimum-range noise. It can
still select a nearer non-person object inside the angular window; there is no
semantic classification of LiDAR points.

## 6. Transform into odom

The associated point begins in the scan frame:

```math
p_s = [r\cos(\theta_s),\ r\sin(\theta_s),\ 0]^T.
```

The fusion node requests the timestamped transform from the scan frame to the
configured output frame, normally `odom`, and applies

```math
p_o = R(q_{o<-s})p_s + t_{o<-s}.
```

This operation removes the robot's own translation and rotation from the
person trajectory. Finite differences of `p_o` therefore estimate motion in a
fixed local frame instead of mistaking robot ego-motion for pedestrian motion.

The implementation rotates vectors directly with the equivalent quaternion
formula. For vector `v` and quaternion vector/scalar parts `(q_v,q_w)`:

```math
t = 2(q_v \times v), \qquad
R(q)v = v + q_w t + q_v \times t.
```

TF lookup timeout is 0.10 s. If either the camera-to-scan or scan-to-output
transform is unavailable at the detection timestamp, the frame publishes no
fused people.

## 7. Velocity and heading estimation

Motion state is keyed by public track ID and stores the previous odom-frame
position, timestamp, filtered velocity, heading, and observation count.

For consecutive valid positions:

```math
v_x^{raw} = \frac{x_t-x_{t-1}}{\Delta t}, \qquad
v_y^{raw} = \frac{y_t-y_{t-1}}{\Delta t}.
```

If `dt <= 0`, velocity is initialized to zero. If

```math
\sqrt{(v_x^{raw})^2+(v_y^{raw})^2} > 4.0\ \text{m/s},
```

the observation is treated as a range/association outlier. The state position
is reset to the current point, filtered velocity is reset to zero, observation
count is reset to zero, and no person is published for that observation.

Otherwise, velocity uses an exponential moving average with `alpha = 0.35`:

```math
v_t = 0.35\,v_t^{raw} + 0.65\,v_{t-1}.
```

The first accepted observation initializes zero velocity. A track is withheld
until it has at least two accepted observations. This prevents a new 2D ID
from immediately becoming a social-force agent with unverified range.

Heading is updated only when filtered speed exceeds `0.09 m/s`:

```math
\psi_t = \operatorname{atan2}(v_y,v_x).
```

Below that threshold, the previous heading is retained. The output
`people_msgs/Person` stores `(x,y)` in `position`, heading in `position.z`, and
the filtered planar velocity in `(velocity.x, velocity.y)`, matching the
downstream planner's expected convention. Motion states older than 1.0 s are
dropped.

## 8. Duplicate fused-person suppression

Multiple image detections can associate with the same physical LiDAR surface.
After fusion, candidates are sorted by descending detector confidence, with
original index as the tie-breaker. A candidate is retained only if its
odom-frame planar distance from every already-retained candidate is at least
`0.30 m`:

```math
\sqrt{(x_i-x_j)^2+(y_i-y_j)^2} \ge 0.30.
```

This is confidence-ordered metric-space non-maximum suppression. It prevents
near-duplicate agents from entering the social-force planner, but it may
suppress two real people whose estimated positions are closer than 0.30 m.

The published reliability is the original YOLO confidence. Tags contain the
numeric portion of the public ID, group ID `-1`, and range source
`velodyne_scan`.

## 9. Deterministic identity colors

Colors are diagnostic only and do not affect association. If an ID has a
numeric suffix `n`, its HSV hue is

```math
hue = (n \cdot 0.618033988749895) \bmod 1.
```

Golden-ratio spacing distributes consecutive IDs around the hue circle. For
an ID without a numeric suffix, the first 64 bits of SHA-256 are interpreted as
an unsigned integer and divided by `2^64` to obtain the hue. Saturation is
`0.85`, value is `0.95`, and the result is converted to 8-bit OpenCV BGR.

Consequently, one public ID has the same color across frames and runs. An ID
switch causes a visible color change. Color is not a proof of physical
identity, and the finite display gamut does not guarantee perceptual separation
for every possible pair of IDs. Recorded baseline tracks remain fixed cyan to
keep their provenance visually distinct.

## 10. Offline tracking evaluation

The evaluator provides automated diagnostics and review candidates. It does
not produce identity ground truth.

### 10.1 Box matching

For boxes `A` and `B`, intersection over union is

```math
IoU(A,B) = \frac{|A \cap B|}{|A|+|B|-|A \cap B|}.
```

All current/baseline pairs in a matched frame are ranked by descending IoU.
Pairs are greedily accepted one-to-one while `IoU >= 0.30`. A baseline frame
must be within 0.075 s of the current frame.

The bag topic `/tracks_insta360_x4_image_raw_compressed` is only a recorded
tracker baseline. It is not human annotation and must not be used to claim
absolute accuracy, recall, MOTA, HOTA, or confirmed ID-switch count.

### 10.2 Intrinsic event heuristics

Center distances below are normalized by the nominal image dimensions
`2880 x 1440` and are ordinary Euclidean distances, not circular panorama
distances.

- **Same-ID spatial jump:** the same public ID moves by at least `0.20`
  normalized image units within 0.5 s.
- **Suspected fragmentation:** a newly observed public ID appears within
  `0.12` normalized units of an inactive old ID last seen no more than 1.5 s
  earlier.
- **Within-ID temporal gap:** consecutive observations of one ID are separated
  by more than `max(0.5 s, 2.5 * median_frame_period)`.
- **Baseline association change:** one baseline ID is greedily matched to a
  different current ID within 0.75 s.

These thresholds identify moments for human review. They can produce false
positives at the panorama seam, during occlusion, or in close crowds. The
generated `review_labels.csv` intentionally leaves verdict, reviewer, and
notes blank; a suspected event is not a confirmed ID switch until reviewed.

Reported descriptive statistics include observations and lifetime per public
ID, singleton/short IDs, temporal gaps, mean matched IoU, current detection
match fraction, and baseline detection match fraction.

### 10.3 Corpus controls

Corpus runs record model path, ReID path, tracker config, input size,
confidence threshold, IoU threshold, playback rate, and target FPS in
`run_config.json`. Resuming with a different configuration is rejected so one
result directory cannot silently mix experiments.

The standard runner uses an isolated ROS domain, records only detector outputs,
and replays only the image topic. Long bags are represented by deterministic
manifest windows; tune and holdout splits remain explicit. The output includes
metrics JSON, candidate-event CSV, review video, blank human-review CSV, and
logs for each segment.

## 11. Reproduction procedure

### 11.1 Install and prepare models on Jetson

```bash
cd /path/to/ScoutMini/ros2_ws/src/scoutmini_social_perception
./scripts/install_jetson_dependencies.sh
./scripts/prepare_models.sh
```

### 11.2 Build

```bash
source /opt/ros/humble/setup.bash
source /path/to/adascore_ws/install/setup.bash
cd /path/to/ScoutMini/ros2_ws
colcon build --symlink-install --packages-up-to scoutmini_social_navigation
source install/setup.bash
```

### 11.3 Run the detector independently

```bash
ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=${HOME}/models/yolo/yolo11n_960.engine \
  reid_model_path:=${HOME}/models/yolo/yolo26n-cls.pt \
  image_topic:=/insta360_x4/image_raw/compressed \
  input_type:=compressed \
  device:=0 \
  target_fps:=8.0 \
  imgsz:=960
```

Inspect:

```bash
ros2 topic echo /people/tracks_2d
ros2 topic echo /people/detector_diagnostics
```

### 11.4 Run range fusion

```bash
ros2 launch scoutmini_social_navigation people_fusion.launch.py \
  output_frame:=odom \
  bearing_offset_rad:=0.0
```

Before treating output range or velocity as physically valid, calibrate and
verify `360_link -> velodyne`, confirm the panorama forward center and bearing
sign, and confirm timestamp alignment under actual sensor load.

### 11.5 Reproduce a corpus evaluation

```bash
ros2 run scoutmini_social_perception build_tracking_corpus \
  --root /path/to/bags \
  --output /path/to/results/corpus_manifest.json \
  --csv /path/to/results/corpus_manifest.csv \
  --only-insta360

ros2 run scoutmini_social_perception run_tracking_corpus \
  --manifest /path/to/results/corpus_manifest.json \
  --output-dir /path/to/results/tracking \
  --rate 0.25 \
  --target-fps 8.0 \
  --imgsz 960 \
  --confidence-threshold 0.35 \
  --iou-threshold 0.45 \
  --continue-on-error
```

Archive the manifest, `run_config.json`, output metrics, logs, source-weight
hashes, Git commit, ROS distribution, JetPack/TensorRT versions, and bag
checksums with any reported result.

## 12. Known limitations affecting interpretation

1. The GMU bags used for offline validation do not provide verified
   camera-to-LiDAR calibration. Their fused range overlays exercise the
   interface but are explicitly unverified.
2. The fusion algorithm associates by horizontal bearing only. It does not use
   detection vertical position, LiDAR point height, or camera-to-LiDAR
   translation in ray selection.
3. Velocity inherits range noise, angular quantization, TF error, and ID error.
   The EMA reduces noise but introduces lag.
4. Public-ID repair is greedy and detector-order-dependent. It is not a global
   multi-frame assignment solver.
5. The recorded tracker is a descriptive baseline, not ground truth. Only
   human-reviewed events or separately annotated identity labels can support
   claims about actual ID switches.
6. TensorRT engine bytes are not expected to be reproducible across different
   TensorRT, CUDA, GPU, or builder versions even when source weights and export
   parameters match.

## 13. Implementation references

- Detector node: [`yolo_people_detector_node.py`](../ros2_ws/src/scoutmini_social_perception/scoutmini_social_perception/yolo_people_detector_node.py)
- BoT-SORT configuration: [`botsort.yaml`](../ros2_ws/src/scoutmini_social_perception/config/botsort.yaml)
- Public-ID reconciliation: [`track_schema.py`](../ros2_ws/src/scoutmini_social_perception/scoutmini_social_perception/track_schema.py)
- Stable visualization colors: [`track_colors.py`](../ros2_ws/src/scoutmini_social_perception/scoutmini_social_perception/track_colors.py)
- Scan fusion node: [`scan_people_fusion_node.py`](../ros2_ws/src/scoutmini_social_navigation/scoutmini_social_navigation/scan_people_fusion_node.py)
- Pure fusion geometry: [`fusion.py`](../ros2_ws/src/scoutmini_social_navigation/scoutmini_social_navigation/fusion.py)
- Tracking evaluator: [`tracking_evaluator.py`](../ros2_ws/src/scoutmini_social_perception/scoutmini_social_perception/tracking_evaluator.py)
- Model preparation: [`prepare_models.sh`](../ros2_ws/src/scoutmini_social_perception/scripts/prepare_models.sh)
- Corpus runner: [`corpus_runner.py`](../ros2_ws/src/scoutmini_social_perception/scoutmini_social_perception/corpus_runner.py)
