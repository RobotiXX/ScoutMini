# ZED RTSP/WebRTC Streaming

The `scoutmini_streaming` ROS 2 package owns the ZED camera streaming workflow.
It starts the ZED wrapper, publishes the camera feed through `image2rtsp`, and
optionally exposes the RTSP stream through a MediaMTX WebRTC browser endpoint.

Use placeholder addresses in commands and docs. Replace `<robot_ip>` with the
active robot LAN address or Tailscale address from `hostname -I` or
`tailscale ip -4`.

## Quick Start

Build the workspace with submodules:

```bash
git submodule update --init --recursive
./scripts/build_ros2_ws.sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

Start the ZED wrapper and `image2rtsp` on the robot:

```bash
ros2_ws/src/scoutmini_streaming/scripts/start_zed_rtsp_stack.sh
```

View the RTSP stream from a reachable laptop:

```bash
ros2_ws/src/scoutmini_streaming/test/view_zed_rtsp_lan.sh <robot_ip>
```

Use SSH tunneling when direct access to port `8554` is unavailable:

```bash
ros2_ws/src/scoutmini_streaming/test/view_zed_rtsp_ssh_tunnel.sh nvidia@<robot_ip>
```

## Wireless LAN

The robot may expose more than one address depending on whether it is connected
to a lab router, hotspot, Ethernet, or Tailscale. Confirm the reachable address
on the robot before sharing viewer commands:

```bash
hostname -I
tailscale ip -4
```

The RTSP URL format is:

```text
rtsp://<robot_ip>:8554/zed
```

Do not expose RTSP or WebRTC ports to the public internet.

## WebRTC Browser Viewer

Install the local MediaMTX test binary only after robot-owner approval:

```bash
ros2_ws/src/scoutmini_streaming/test/install_mediamtx_local.sh
```

Start RTSP plus the WebRTC gateway:

```bash
ros2_ws/src/scoutmini_streaming/scripts/start_zed_webrtc_stack.sh
```

Then open:

```text
http://<robot_ip_or_tailscale_ip>:8889/zed/
```

If the RTSP stream is already running, start only the WebRTC gateway:

```bash
START_RTSP=0 ros2_ws/src/scoutmini_streaming/scripts/start_zed_webrtc_stack.sh
```

MediaMTX configuration lives in:

```text
ros2_ws/src/scoutmini_streaming/config/mediamtx_zed_webrtc.yml
```

If WebRTC loads but video does not connect across a private network, confirm
TCP reachability to `8889`, check whether UDP `8189` is blocked, then set
`webrtcAdditionalHosts` to the private robot IP if ICE candidates are wrong.

## Checks and Diagnostics

Check dependencies, ports, and ROS topic visibility:

```bash
ros2_ws/src/scoutmini_streaming/scripts/check_zed_stream_stack.sh
```

Collect a support bundle:

```bash
ros2_ws/src/scoutmini_streaming/scripts/collect_zed_stream_diagnostics.sh
```

Record a short ZED debug bag:

```bash
DURATION_SECONDS=60 ros2_ws/src/scoutmini_streaming/test/record_zed_debug_bag.sh
```

Stop streaming processes:

```bash
ros2_ws/src/scoutmini_streaming/scripts/stop_zed_stream_stack.sh
```

From a viewer laptop, test RTSP connectivity:

```bash
nc -vz <robot_ip> 8554
```

## Permissions

- Runtime start/stop does not require `sudo`.
- Any future autostart setup requires robot-owner approval.
- The robot user must be able to access the ZED camera.
- Fresh setup may need `sudo` for ROS, GStreamer, RTSP, or MediaMTX dependency
  installation.
- Viewers need network reachability to the selected robot address.
