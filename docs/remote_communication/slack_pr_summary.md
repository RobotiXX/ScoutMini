# Slack Stream Control PR Summary

## Purpose

Add a contained Slack-based communication prototype for the Scout robot:

- Report robot network/status information in Slack.
- Share the ZED WebRTC stream URL.
- Start, stop, and inspect only the ZED RTSP/WebRTC stream stack from Slack.
- Keep robot motion, Nav2, teleop, mapping, task execution, and raw shell
  execution out of the Slack command surface.

This PR should land after the ZED WebRTC foundation PR, which owns
`scripts/zed_rtsp/` and `docs/zed_rtsp_streaming.md`.

## Included Areas

- `ros2_ws/src/scoutmini_tasks/scoutmini_tasks/slack_status_poster.py`
- `ros2_ws/src/scoutmini_tasks/scoutmini_tasks/slack_command_bot.py`
- `ros2_ws/src/scoutmini_tasks/setup.py`
- `ros2_ws/src/scoutmini_tasks/README.md`
- `scripts/slack/`
- `docs/remote_communication/`
- `.gitignore`

## Deliberately Out of Scope

- Nav2 configuration and launch behavior.
- Route-loop behavior changes.
- Map, route, or waypoint edits.
- Robot base control.
- Eduroam provisioning.
- Systemd/autostart setup for the Slack bot.
- Public internet exposure of RTSP/WebRTC ports.
- ZED WebRTC implementation internals, which belong to the foundation PR.

## Slack Commands

```text
status
stream
stream status
stream start
stream stop
diagnostics
help
```

`stream start`, `stream stop`, and `diagnostics` call:

```bash
scripts/slack/control_zed_stream.sh
```

That script is the only Slack-facing process-control path. It manages the ZED
RTSP/WebRTC stack and nothing else.

## Secrets

Slack tokens are not committed. Local secrets live in:

```text
.local/secrets/slack.env
```

The template is:

```text
docs/remote_communication/slack.env.example
```

## Validation Run

```bash
python3 - <<'PY'
import ast
from pathlib import Path
for path in [
    Path('ros2_ws/src/scoutmini_tasks/scoutmini_tasks/slack_command_bot.py'),
    Path('ros2_ws/src/scoutmini_tasks/scoutmini_tasks/slack_status_poster.py'),
]:
    ast.parse(path.read_text(), filename=str(path))
    print(f'ok: {path}')
PY

bash -n \
  scripts/slack/control_zed_stream.sh \
  scripts/slack/start_slack_bot.sh

source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select scoutmini_tasks
```

Live stream-control validation:

```bash
cd /home/nvidia/repos/ScoutMini
./scripts/slack/control_zed_stream.sh status
./scripts/slack/control_zed_stream.sh start
curl -L --max-time 5 -o /tmp/zed_viewer_check.html \
  -w '%{http_code} %{content_type}\n' \
  http://127.0.0.1:8889/zed/
./scripts/slack/control_zed_stream.sh stop
```

Expected result:

- start opens `8554`, `8889`, and `8189`.
- viewer endpoint returns `200 text/html`.
- stop closes the stream ports.

## Known Local State to Exclude

These are not part of the Slack PR:

- ZED WebRTC foundation files from the first PR:
  `scripts/zed_rtsp/` and `docs/zed_rtsp_streaming.md`
- `ros2_ws/src/scoutmini_tasks/scoutmini_tasks/route_loop_runner.py`
- `ros2_ws/src/scoutmini_tasks/config/route_loop_runner.yaml`
- `ros2_ws/src/scoutmini_bringup/launch/basic.launch.py`
- `ros2_ws/src/scoutmini_nav2/config/nav2.yaml`
- `ros2_ws/src/scoutmini_nav2/launch/navigation.launch.py`
- `ros2_ws/src/map_tools/maps/fuse_3rd/`
- submodule working tree changes
- local generated files such as `ZED_Diagnostic_Results.json` and `qt_env/`
