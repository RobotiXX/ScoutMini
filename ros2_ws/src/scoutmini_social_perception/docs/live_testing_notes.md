# Live Testing Notes

These notes summarize initial live tests with the Insta360 X4 and YOLO people detector on the ScoutMini robot.

## Current Startup Path

Camera:

```bash
cd /home/nvidia/repos/ScoutMini/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch insta360_ros_driver x4_equirectangular_crop_h264_cuda.launch.xml
```

YOLO:

```bash
cd /home/nvidia/repos/ScoutMini/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scoutmini_social_perception yolo_people_pipeline.launch.py \
  model_path:=/home/nvidia/models/yolo/yolo11n.pt \
  target_fps:=1.0
```

Debug image:

```bash
cd /home/nvidia/repos/ScoutMini/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /people/debug_image
```

## Observations

- Live person detection works on the robot with `/people/detections_2d` and `/people/debug_image`.
- The equirectangular seam appears to correspond to the back side of the robot, not the front. That is acceptable and likely desired.
- The current CPU path is slow. With detector metrics around 280 ms per inference, the realistic CPU processing target is roughly 1-3 FPS.
- `target_fps:=1.0` is stable for testing. `target_fps:=3.0` is the next reasonable CPU tuning point.
- The image conversion path publishes `/equirectangular/image` around 10-15 FPS when the camera is connected in Android Control mode.

## Next Tuning Steps

- Use the `imgsz` launch argument so CPU tests can compare 640, 512, and 416.
- Keep seam shifting low priority unless later tests show the front direction is split.
- Confirm `bearing_rad` by standing front, left, right, and back relative to the robot.
- Plan for CUDA PyTorch or TensorRT before expecting real-time social navigation performance.

See `adascore_readiness.md` for the gated path from perception tuning to AdaSCoRe integration.
