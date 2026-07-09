# Scout Slack Gateway

The Slack gateway is a ROS 2 node that connects Slack Socket Mode to ScoutMini.
It keeps Slack credentials in `.local/secrets/slack.env`, exposes a ROS service
for outgoing Slack messages, and publishes unknown Slack requests for future
ROS-side handling.

## Setup

1. Copy `docs/remote_communication/slack.env.example` to
   `.local/secrets/slack.env`.
2. Fill in `SLACK_BOT_TOKEN` and `SLACK_APP_TOKEN`.
3. Build the ROS workspace:

   ```bash
   cd ros2_ws
   colcon build --symlink-install --packages-select scoutmini_interfaces scoutmini_tasks
   ```

4. Start the gateway:

   ```bash
   ./scripts/slack/start_slack_gateway.sh
   ```

## ROS API

- Node: `slack_gateway`
- Service: `/scout/slack/send_message`
- Type: `scoutmini_interfaces/srv/SendSlackMessage`
- Topic: `/scout/slack/command_request`
- Topic type: `std_msgs/msg/String` containing JSON

The gateway handles `help`, `status`, `stream`, `stream status`,
`stream start`, `stream stop`, and `diagnostics`. It does not execute motion,
navigation, route, `cmd_vel`, or arbitrary shell commands.

Outgoing Slack messages sent through `/scout/slack/send_message` are rate
limited in-process. Defaults are one message every two seconds per channel with
a burst of three, and one message per second globally with a burst of ten.
