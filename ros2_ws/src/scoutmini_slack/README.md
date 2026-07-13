# ScoutMini Slack

`scoutmini_slack` owns the Slack integration for ScoutMini. It keeps Slack
credentials in a local env file, exposes a ROS service for outgoing Slack
messages, and publishes incoming Slack event metadata for ROS-side handling.

## Setup

1. Copy the env template:

   ```bash
   mkdir -p .local/secrets
   cp ros2_ws/src/scoutmini_slack/config/slack.env.example .local/secrets/slack.env
   chmod 600 .local/secrets/slack.env
   ```

2. Fill in `SLACK_BOT_TOKEN`, `SLACK_APP_TOKEN`, and optional defaults.
3. Build the affected ROS packages:

   ```bash
   cd ros2_ws
   colcon build --symlink-install --packages-select scoutmini_interfaces scoutmini_slack
   ```

4. Start the gateway:

   ```bash
   ros2_ws/src/scoutmini_slack/scripts/start_slack_gateway.sh
   ```

## ROS API

- Node: `slack_gateway`
- Service: `/scout/slack/send_message`
- Service type: `scoutmini_interfaces/srv/SendSlackMessage`
- Topic: `/scout/slack/incoming_message`
- Topic: `/scout/slack/command_request`
- Topic type: `std_msgs/msg/String` containing JSON

Every supported Slack event publishes an incoming-message JSON payload with:

```json
{
  "source": "slack",
  "channel": "C...",
  "channel_type": "channel|group|im",
  "user": "U...",
  "text": "message text",
  "ts": "slack message timestamp",
  "thread_ts": "slack thread timestamp",
  "event_type": "app_mention|message",
  "handled": true
}
```

Unrecognized requests are also published to `/scout/slack/command_request` with
the same source metadata and `handled: false`.

The gateway handles `help`, `status`, `stream`, `stream status`,
`stream start`, `stream stop`, and `diagnostics`. It does not execute motion,
navigation, route, `cmd_vel`, or arbitrary shell commands.

Outgoing Slack messages sent through `/scout/slack/send_message` are rate
limited in-process. Defaults are one message every two seconds per channel with
a burst of three, and one message per second globally with a burst of ten.
