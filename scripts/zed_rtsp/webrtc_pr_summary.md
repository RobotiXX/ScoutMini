# ZED WebRTC Remote Camera PR Summary

## Purpose

Add the remote camera access foundation for the Scout robot:

- Keep the existing ZED ROS topic to RTSP path.
- Add MediaMTX as an RTSP-to-WebRTC gateway.
- Make the ZED stream viewable in a browser at `/zed/`.
- Support off-LAN private access through Tailscale without public ports.
- Add diagnostics, smoke tests, and local install instructions for repeatable
  setup.

## Architecture

```text
ZED ROS compressed image topic
  -> image2rtsp
  -> RTSP on :8554
  -> MediaMTX
  -> WebRTC browser viewer on :8889/zed/
  -> Tailscale/private network access
```

## Included Areas

- `scripts/zed_rtsp/README.md`
- `scripts/zed_rtsp/start_zed_webrtc_stack.sh`
- `scripts/zed_rtsp/stop_zed_stream_stack.sh`
- `scripts/zed_rtsp/mediamtx_zed_webrtc.yml`
- `scripts/zed_rtsp/install_mediamtx_local.sh`
- `scripts/zed_rtsp/check_zed_stream_stack.sh`
- `scripts/zed_rtsp/collect_zed_stream_diagnostics.sh`
- `scripts/zed_rtsp/smoke_test_zed_webrtc_stack.sh`
- `scripts/zed_rtsp/record_zed_debug_bag.sh`
- `scripts/zed_rtsp/viewer/zed_webrtc_viewer.html`
- `scripts/zed_rtsp/*/.gitignore` placeholders for local outputs/tools.
- `docs/zed_rtsp_streaming.md`

## Deliberately Out of Scope

- Slack commands and Slack tokens.
- Robot motion, Nav2, teleop, mapping, and task execution.
- Public internet exposure of stream ports.
- Committing downloaded MediaMTX binaries or generated logs.
- Systemd/autostart setup.

## Validation

```bash
cd /home/nvidia/repos/ScoutMini

bash -n \
  scripts/zed_rtsp/start_zed_webrtc_stack.sh \
  scripts/zed_rtsp/stop_zed_stream_stack.sh \
  scripts/zed_rtsp/check_zed_stream_stack.sh \
  scripts/zed_rtsp/collect_zed_stream_diagnostics.sh \
  scripts/zed_rtsp/record_zed_debug_bag.sh \
  scripts/zed_rtsp/smoke_test_zed_webrtc_stack.sh

./scripts/zed_rtsp/smoke_test_zed_webrtc_stack.sh
```

Expected live behavior:

- RTSP listens on TCP `8554`.
- WebRTC viewer listens on TCP `8889`.
- MediaMTX ICE listens on UDP `8189`.
- `http://<robot-ip-or-tailscale-ip>:8889/zed/` opens the browser viewer.
- `stop_zed_stream_stack.sh` closes all stream ports.
