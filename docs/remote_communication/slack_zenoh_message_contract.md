# Slack and Zenoh Message Contract

This is the first communication contract for user/robot messages. It keeps
human requests separate from robot execution and avoids exposing raw motion
control.

## Layers

```text
Slack
  human-facing commands, approvals, status, help messages

Robot task gateway
  validates requests and converts accepted work into ROS-safe actions

Zenoh
  robot/backend/robot low-rate structured messages

ROS 2
  local robot execution, Nav2, perception, diagnostics
```

## Slack Commands

Initial Slack commands should be dry-run/status only:

- `status`: return robot online state, active network, Tailscale IP, and stream
  URL.
- `stream`: return the current WebRTC viewer URL.
- `request_help`: notify the channel that the robot needs human help.
- `request_task`: submit a high-level task request, but do not trigger motion
  in the first milestone.

Do not expose `cmd_vel`, teleop, Nav2 action calls, or shell execution through
Slack.

## First Slack Prototype

The first implementation is one-way status posting from the robot to Slack. It
uses `SLACK_BOT_TOKEN` and `SLACK_CHANNEL_ID` from the environment and posts:

- hostname
- robot IPs
- Tailscale IP
- ZED WebRTC stream state
- ZED WebRTC stream URL when online
- stream start command when offline
- currently open stream ports
- explicit note that motion commands are disabled

Run after building/sourcing the ROS workspace:

```bash
export SLACK_BOT_TOKEN='xoxb-...'
export SLACK_CHANNEL_ID='C...'
ros2 run scoutmini_tasks slack_status_poster
```

Install the Slack SDK on the robot with pip, since the Ubuntu apt repositories
configured on this robot do not provide a `python3-slack-sdk` package:

```bash
python3 -m pip install slack_sdk
```

Keep the token out of git and chat logs.

## Slack Command Bot

The next implementation is a long-running read-only Socket Mode bot. It uses
`SLACK_BOT_TOKEN` and `SLACK_APP_TOKEN` from the environment and responds to
app mentions or direct messages:

- `status`: same robot status payload as the one-way poster.
- `stream`: current ZED WebRTC URL and online/offline state.
- `help`: supported commands.

Run after building/sourcing the ROS workspace:

```bash
export SLACK_BOT_TOKEN='xoxb-...'
export SLACK_APP_TOKEN='xapp-...'
ros2 run scoutmini_tasks slack_command_bot
```

Or use the manual launcher, which sources the ignored local env file:

```bash
cd /home/nvidia/repos/ScoutMini
./scripts/slack/start_slack_bot.sh
```

The Slack app needs Socket Mode enabled and event subscriptions for app
mentions plus direct messages. The bot still does not start streams, execute
shell commands, send Nav2 goals, teleop the base, or publish velocity commands.

For repeated testing, copy the template into the ignored local secrets area:

```bash
mkdir -p /home/nvidia/repos/ScoutMini/.local/secrets
cp /home/nvidia/repos/ScoutMini/docs/remote_communication/slack.env.example \
  /home/nvidia/repos/ScoutMini/.local/secrets/slack.env
chmod 600 /home/nvidia/repos/ScoutMini/.local/secrets/slack.env
```

Then edit `.local/secrets/slack.env` locally and load it with:

```bash
source /home/nvidia/repos/ScoutMini/.local/secrets/slack.env
```

## Common Message Fields

Use these fields for Slack payloads, Zenoh messages, and robot task gateway
logs:

```yaml
message_id: string
timestamp: ISO-8601 string
source: string
target_robot: string
type: status | task_request | task_status | help_request
task_id: string
state: received | accepted | rejected | running | completed | failed | needs_help
summary: string
details: object
```

## First Zenoh Topics

Use low-rate topics only:

```text
scout/status/heartbeat
scout/task/request
scout/task/status
scout/help/request
```

Do not bridge image, depth, point cloud, or high-rate debug topics through
Zenoh by default.

## Safety Policy

- Slack and Zenoh can request work; they do not directly control the base.
- Robot-side validation is required before any ROS action is called.
- Motion tasks are rejected unless the robot is explicitly in supervised mode.
- First implementation accepts only status and dry-run task requests.
- Physical supervision is required for any Nav2 route execution or social
  navigation test.
