# ScoutMini Streaming

`scoutmini_streaming` owns the ZED ROS-image-to-RTSP bridge and the MediaMTX
WebRTC gateway. The ZED wrapper remains owned by normal robot bringup.

## Runtime

Build and source the workspace, then start the camera and RTSP bridge through
the robot bringup, `scripts/nav2_byobu`, or `scripts/mapping_byobu`. For a
camera-only test, use:

```bash
ros2_ws/src/scoutmini_streaming/test/start_standalone_zed_rtsp.sh
```

Install MediaMTX and the user service once:

```bash
ros2_ws/src/scoutmini_streaming/test/install_mediamtx_local.sh
ros2_ws/src/scoutmini_streaming/scripts/install_user_service.sh
```

The installer loads but does not enable the service. Start it manually after
the camera and RTSP bridge are healthy:

```bash
systemctl --user start scoutmini-webrtc.service
```

RTSP is available at `rtsp://<robot_ip>:8554/zed`. The WebRTC viewer is
available at `http://<robot_ip_or_tailscale_ip>:8889/zed/`.

## Health And Diagnostics

```bash
ros2_ws/src/scoutmini_streaming/scripts/stream_health.sh --status
ros2_ws/src/scoutmini_streaming/scripts/check_zed_stream_stack.sh
ros2_ws/src/scoutmini_streaming/scripts/collect_zed_stream_diagnostics.sh
```

Health requires an actual compressed ROS image, an H.264 RTSP video track, and
a successful WebRTC HTTP response. Open ports alone are not considered healthy.

Runtime state is stored under `$XDG_RUNTIME_DIR/scoutmini`; MediaMTX is under
`~/.local/lib/scoutmini`; diagnostics and bags are under
`~/.local/state/scoutmini`. No runtime state is written into the package.

These tools do not start navigation, mapping, teleoperation, or base motion.
