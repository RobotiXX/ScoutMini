# Scout Remote Access and Communication Tradeoffs

This note summarizes the remote access direction from Nhat's project prompt and
the tradeoffs discussed during the first feasibility pass.

## Project Direction

Near-term need:

- Stream at least one Scout camera, starting with the ZED, to a browser from
  outside the local robot network.
- Make SSH into the robot possible from anywhere for approved developers.
- Prefer TCP where it is a natural fit, but keep low-latency video usable.

Ultimate communication goals:

- Robots connect through eduroam where possible for broad campus coverage.
- Users can access at least one Scout camera stream from anywhere.
- Users and robots can relay messages to each other:
  - users request robot tasks or status
  - robots request help from users or other robots
- Evaluate Python Slack SDK for human-facing messages.
- Evaluate Tailscale plus Zenoh for private robot/backend/robot messaging.

Robot/autonomy internship track:

- Build YOLO plus tracking for people detection on the ROS 2 system.
- Prefer real ZED/360 camera bags before live navigation integration.
- Feed tracked people into AdaSCoRe/social navigation.
- Test navigation behavior only with physical supervision.

## Current Verified State

- Robot Tailscale hostname: `scoutmini-zed`
- Robot Tailscale IP: `100.78.242.13`
- Tailscale is installed, running, enabled, and authenticated.
- OpenSSH server is installed, running, and enabled.
- ZED WebRTC browser playback works over Tailscale:

```text
http://100.78.242.13:8889/zed/
```

- Current working video path:

```text
ZED ROS 2 compressed image topic
  -> image2rtsp
  -> RTSP on TCP :8554
  -> MediaMTX
  -> WebRTC browser endpoint on TCP :8889 plus ICE/media on UDP :8189
  -> Tailscale private network
```

- Current active Wi-Fi observed during setup was `TP-Link_B916`.
- Eduroam remains a future uplink option, but the first attempt hit
  privilege/access issues and should be paused until the correct university
  credential/certificate flow is available.

## Option Matrix

| Option | Best use | Pros | Cons | Recommendation |
| --- | --- | --- | --- | --- |
| Tailscale + WebRTC | Private browser camera viewing and SSH from anywhere | Already works; avoids public ports; encrypted tailnet; simple developer access; works over any internet uplink | Viewers need tailnet access; WebRTC prefers UDP for media; browser shows "Not secure" because the viewer is plain HTTP inside the tailnet | Use as the primary near-term path |
| Eduroam as uplink | Broad campus coverage if the robot can authenticate | Keeps the same Tailscale/SSH/WebRTC model once connected; no public ports | Requires correct university credentials/certificates; current attempt hit access/privilege issues | Leave as a future option |
| Direct eduroam/campus SSH | SSH directly to robot campus IP | No extra overlay if campus routing allows it | Often blocked by NAT, firewalling, or wireless client isolation; weaker access boundary; IPs may change | Do not pursue for now |
| Public SSH gateway or reverse tunnel | Access without joining tailnet | Flexible for demos or temporary users | Requires public host, account policy, logs, hardening, key management, and ownership | Defer until dev-team workflow is stable |
| RTSP over LAN or SSH tunnel | Developer fallback viewing | Simple; works with VLC/ffplay; RTSP source path already exists | Poor browser support; direct LAN only unless tunneled; not ideal for non-technical users | Keep as debugging fallback |
| WebRTC over UDP ICE | Low-latency browser video | Best fit for live camera; tolerates packet loss better than TCP retransmission delay | UDP may be blocked or relayed on some networks | Keep as default video media path |
| WebRTC over TCP ICE | Restrictive network fallback | More firewall-friendly; aligns with "TCP wherever possible" | Less efficient for real-time video; congestion can create progressive delay | Test only if UDP media fails on the chosen uplink |
| HLS or low-latency HLS over HTTP/TCP | Firewall-friendly video fallback | Browser/CDN friendly; TCP-only HTTP path | Higher latency than WebRTC; not ideal for teleoperation-like viewing | Consider if WebRTC is unreliable on the chosen uplink |
| Slack Python SDK | Human-facing commands/status | Good for user requests, task approval, status, and help messages; no ROS exposure to users | Requires Slack app/token setup; not suitable for raw robot control | Use for human-facing layer after recovery hardening |
| Zenoh ROS 2 bridge | Robot/backend/robot structured messaging | Supports ROS topics/services/actions across hosts; useful over Tailscale; can use TCP | Needs careful DDS isolation to avoid duplicate/looping traffic; more complex than Slack/status scripts | Evaluate after basic remote ops are stable |
| Public WebRTC/Funnel/reverse proxy | Camera access without tailnet | Easier for arbitrary viewers | Auth, HTTPS, exposure, TURN/proxy costs, institutional security review | Defer unless project needs non-tailnet viewers |

## TCP vs UDP Policy

Use TCP wherever it naturally fits:

- SSH
- Slack and web APIs
- diagnostics endpoints
- route/task/status messages
- RTSP source pull from MediaMTX to image2rtsp
- Zenoh control/status experiments if configured that way

Use UDP where low-latency media benefits from it:

- Tailscale direct WireGuard path, when available
- WebRTC ICE/media for live camera viewing

Reasoning: TCP retransmission is good for commands, files, and logs, but it can
turn video packet loss into growing delay. For camera viewing, late frames are
usually worse than dropped frames.

## Recommended Architecture

Use the current working network first. Treat eduroam, lab LAN, and travel
routers as interchangeable uplink options. Keep Tailscale as the private access
and identity layer.

```text
Robot network uplink
  current working Wi-Fi / lab LAN / travel router / future eduroam

Private access layer
  Tailscale tailnet
  SSH to scoutmini-zed
  WebRTC camera URL over tailnet

Streaming layer
  ZED/360 camera
  ROS 2 image topic
  RTSP fallback
  WebRTC primary browser viewer

Remote ops layer
  start/stop/check scripts
  bounded smoke tests
  diagnostics bundle
  ROS bag recording helpers

Human messaging layer
  Slack status/task/help interface
  no raw velocity commands

Robot messaging layer
  Zenoh evaluation for low-rate status/task/help topics
  no high-bandwidth image topics by default

Autonomy/perception layer
  YOLO + tracker
  /person_tracks
  AdaSCoRe/social navigation adapter
  simulation and bag replay before supervised robot motion
```

## Feasibility Study Plan

### Phase A: Current Baseline

Record the current known-good state:

```bash
tailscale status
tailscale ip -4
tailscale netcheck
systemctl status ssh --no-pager
systemctl status tailscaled --no-pager
./scripts/zed_rtsp/check_zed_stream_stack.sh
```

Success criteria:

- `tailscaled` is running.
- SSH service is running.
- Robot has a Tailscale IP.
- WebRTC ZED stream still works over Tailscale.

### Phase B: Uplink Expansion Test

With someone physically near the robot:

1. Keep the current working Wi-Fi profile available as fallback.
2. Connect robot to the candidate uplink using the approved local flow.
3. Confirm outbound internet works.
4. Confirm Tailscale reconnects.
5. From an off-LAN laptop, test:

```bash
tailscale ping scoutmini-zed
ssh nvidia@100.78.242.13
```

Success criteria:

- SSH works from outside the robot LAN.
- Tailscale is stable after Wi-Fi changes.
- If direct Tailscale UDP fails, DERP relay still provides usable SSH.

### Phase C: Video Over Selected Uplink + Tailscale

Run a bounded video test:

```bash
cd /home/nvidia/repos/ScoutMini
TEST_SECONDS=300 ./scripts/zed_rtsp/smoke_test_zed_webrtc_stack.sh
```

From the laptop:

```text
http://100.78.242.13:8889/zed/
```

Success criteria:

- Page loads.
- Video plays.
- Latency is acceptable for observation.

If video fails:

1. Confirm TCP `8889` reachability.
2. Check whether UDP `8189` is blocked or relayed.
3. Test WebRTC TCP ICE as a fallback.
4. If needed, evaluate HLS/LL-HLS over HTTP/TCP for observation-only viewing.

### Phase D: Messaging Direction

After SSH/video are stable:

- Use Slack for human commands and status:
  - `status`
  - `stream link`
  - `request task`
  - `robot needs help`
- Use Zenoh only for low-rate robot/backend/robot messages at first:
  - heartbeat
  - task request
  - task status
  - help request
- Do not bridge camera topics through Zenoh by default.
- Do not expose raw velocity commands through Slack, browser UI, or Zenoh.

## Safety and Scope Boundaries

- Remote SSH and camera streaming are acceptable remote work.
- Perception development can mostly happen remotely using recorded bags.
- Any robot motion, Nav2 route execution, or social navigation test requires:
  - emergency stop availability
  - clear test area
  - low speed limits
  - physical observer
  - first tests with wheels lifted or robot stationary
  - slow open-space tests before testing around humans
- Some failure modes still require physical intervention:
  - Wi-Fi loss
  - Tailscale disconnect
  - SSH failure
  - Jetson freeze
  - camera driver crash
  - robot stuck
  - hard reboot needed

## Default Recommendation

Proceed with:

1. Current working Wi-Fi as the robot uplink; eduroam remains a future option.
2. Tailscale as the private access layer.
3. SSH over Tailscale for developers.
4. WebRTC over Tailscale for the ZED camera.
5. TCP for commands/status/logs/SSH.
6. UDP for WebRTC media unless the chosen uplink blocks it.
7. Slack for human-facing messages after remote recovery scripts are solid.
8. Zenoh for low-rate robot/backend messaging after Slack/status workflow is
   clear.
9. YOLO/tracking/AdaSCoRe as the later autonomy track, starting from recorded
   bags before physical navigation tests.

## References

- Tailscale connection types: https://tailscale.com/docs/reference/connection-types
- Tailscale SSH: https://tailscale.com/docs/features/tailscale-ssh
- MediaMTX configuration: https://github.com/bluenviron/mediamtx
- Slack Python SDK: https://docs.slack.dev/tools/python-slack-sdk/
- Zenoh ROS 2 DDS bridge: https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds
- AdaSCoRe: https://github.com/maurom3197/adascore
