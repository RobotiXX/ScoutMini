# ZED RTSP And WebRTC Streaming

The production stream is three separately owned processes:

1. Robot bringup starts `zed_wrapper` as camera `zed2` at HD1080/30 FPS.
2. `scoutmini_streaming/image2rtsp.launch.py` consumes
   `/zed2/zed_node/rgb/color/rect/image/compressed` and publishes H.264 RTSP.
3. The manually started `scoutmini-webrtc.service` proxies RTSP to WebRTC.

This separation prevents Slack stream commands from restarting or killing a
camera used by navigation or perception.

## Setup

```bash
git submodule update --init --recursive
./scripts/build_ros2_ws.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2_ws/src/scoutmini_streaming/test/install_mediamtx_local.sh
ros2_ws/src/scoutmini_streaming/scripts/install_user_service.sh
```

The service is installed disabled. Normal camera bringup is provided by
`scoutmini_bringup`, `scripts/nav2_byobu`, and `scripts/mapping_byobu`. A
bounded standalone camera/RTSP test is available under the package's `test/`
directory.

Start WebRTC only after camera and RTSP health pass:

```bash
ros2_ws/src/scoutmini_streaming/scripts/stream_health.sh --status
systemctl --user start scoutmini-webrtc.service
```

Viewer addresses:

```text
rtsp://<robot_ip>:8554/zed
http://<robot_ip_or_tailscale_ip>:8889/zed/
```

Use `hostname -I` and `tailscale ip -4` to select a reachable private address.
Do not expose these ports to the public internet.

## Failure Diagnosis

```bash
ros2_ws/src/scoutmini_streaming/scripts/check_zed_stream_stack.sh
ros2_ws/src/scoutmini_streaming/scripts/collect_zed_stream_diagnostics.sh
journalctl --user -u scoutmini-webrtc.service -n 100 --no-pager
```

The check distinguishes camera frame delivery, H.264 RTSP delivery, and the
WebRTC endpoint. It also reports recent USB/UVC errors because a listed USB
device or ROS topic does not prove that the camera is delivering frames.
