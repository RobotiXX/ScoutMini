# Scout ZED RTSP/WebRTC Tools

This directory owns the camera streaming work for the Scout robot. It is kept
separate from navigation, robot motion, mapping, and task code.

## Main Commands

Start the original RTSP stream:

```bash
ros2_ws/src/scoutmini_streaming/scripts/start_zed_rtsp_stack.sh
```

Start RTSP plus the WebRTC browser gateway:

```bash
ros2_ws/src/scoutmini_streaming/scripts/start_zed_webrtc_stack.sh
```

Run a bounded WebRTC smoke test:

```bash
ros2_ws/src/scoutmini_streaming/test/smoke_test_zed_webrtc_stack.sh
```

Stop the streaming stack:

```bash
ros2_ws/src/scoutmini_streaming/scripts/stop_zed_stream_stack.sh
```

Check local stream dependencies and network state:

```bash
ros2_ws/src/scoutmini_streaming/scripts/check_zed_stream_stack.sh
```

Collect a diagnostics bundle:

```bash
ros2_ws/src/scoutmini_streaming/scripts/collect_zed_stream_diagnostics.sh
```

Record a short ZED debug bag:

```bash
DURATION_SECONDS=60 ros2_ws/src/scoutmini_streaming/test/record_zed_debug_bag.sh
```

Install the local MediaMTX binary under `test/tools/`:

```bash
ros2_ws/src/scoutmini_streaming/test/install_mediamtx_local.sh
```

## Viewer URLs

RTSP:

```text
rtsp://<robot_ip>:8554/zed
```

WebRTC browser viewer:

```text
http://<robot_ip_or_tailscale_ip>:8889/zed/
```

## Files

- `start_zed_rtsp_stack.sh`: launches ZED ROS wrapper and `image2rtsp`.
- `start_zed_webrtc_stack.sh`: launches the RTSP stack and MediaMTX.
- `smoke_test_zed_webrtc_stack.sh`: bounded local WebRTC startup test.
- `stop_zed_stream_stack.sh`: stops the streaming stack wrappers and gateway.
- `collect_zed_stream_diagnostics.sh`: writes a diagnostics bundle.
- `record_zed_debug_bag.sh`: records camera/TF topics for offline debugging.
- `mediamtx_zed_webrtc.yml`: MediaMTX config for RTSP-to-WebRTC.
- `install_mediamtx_local.sh`: reproducible local MediaMTX install.
- `check_zed_stream_stack.sh`: read-only diagnostics.
- `tailscale_robot_setup.md`: private remote access setup notes.
- `viewer/`: optional local browser helper page.
- `diagnostics/`: ignored diagnostics bundle output area.
- `rosbags/`: ignored ROS bag output area for camera/perception debugging.
- `tools/`: ignored local binary/download area.

## Boundaries

These scripts start camera streaming only. They do not start Nav2, teleop,
robot base control, mapping, task execution, or autonomous motion.
