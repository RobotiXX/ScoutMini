# ScoutMini Tasks Package

This ROS 2 package contains high-level task and communication helpers for the
ScoutMini project.

## Current Entry Points

- `route_loop_runner`: sends configured waypoint routes to Nav2.
- `slack_status_poster`: posts robot status, ZED stream state, and the stream
  URL or start command to Slack.
- `slack_command_bot`: listens for Slack `status`, `stream`, `stream start`,
  `stream stop`, `diagnostics`, and `help` requests over Slack Socket Mode.

## Slack Prototype

The Slack prototype is intentionally limited to communication status and ZED
stream control. It does not expose shell execution, Nav2 actions, teleop, raw
velocity commands, task execution, mapping, or autonomous robot motion.

If the WebRTC gateway is offline, the Slack message reports `ZED stream:
offline` and shows the controlled command path for starting the stream.

Secrets are not stored in this package. For local testing, export them in the
terminal:

```bash
export SLACK_BOT_TOKEN='xoxb-...'
export SLACK_CHANNEL_ID='C...'
export SLACK_APP_TOKEN='xapp-...'
```

For repeated use, keep a local ignored env file:

```bash
mkdir -p /home/nvidia/repos/ScoutMini/.local/secrets
cp /home/nvidia/repos/ScoutMini/docs/remote_communication/slack.env.example \
  /home/nvidia/repos/ScoutMini/.local/secrets/slack.env
chmod 600 /home/nvidia/repos/ScoutMini/.local/secrets/slack.env
```

Edit `.local/secrets/slack.env` locally, then load it in each new terminal:

```bash
source /home/nvidia/repos/ScoutMini/.local/secrets/slack.env
```

Then run:

```bash
python3 scoutmini_tasks/slack_status_poster.py
```

For the long-running command responder, enable Socket Mode on the Slack app,
subscribe to `app_mention` and direct-message events, then run:

```bash
ros2 run scoutmini_tasks slack_command_bot
```

For normal manual use on the robot, prefer the repo-level launcher so secrets
come from the ignored local env file:

```bash
cd /home/nvidia/repos/ScoutMini
./scripts/slack/start_slack_bot.sh
```

Supported commands are:

```text
status
stream
stream status
stream start
stream stop
diagnostics
help
```

`stream start` and `stream stop` call the repo-level supervisor:

```bash
/home/nvidia/repos/ScoutMini/scripts/slack/control_zed_stream.sh
```

That supervisor only manages the ZED RTSP/WebRTC stream stack.
