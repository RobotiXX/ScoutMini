# ScoutMini Slack

`scoutmini_slack` provides one canonical Socket Mode gateway. It publishes
incoming Slack metadata to ROS, exposes a rate-limited outgoing-message
service, and delegates WebRTC start/stop to `scoutmini-webrtc.service`.

## Setup

```bash
mkdir -p .local/secrets
cp ros2_ws/src/scoutmini_slack/config/slack.env.example .local/secrets/slack.env
chmod 600 .local/secrets/slack.env
cd ros2_ws
colcon build --symlink-install --packages-select scoutmini_interfaces scoutmini_streaming scoutmini_slack
cd ..
ros2_ws/src/scoutmini_slack/scripts/install_user_service.sh
```

Fill in `SLACK_BOT_TOKEN` and `SLACK_APP_TOKEN`. The service installer loads
but does not enable the unit. Start it manually with:

```bash
systemctl --user start scoutmini-slack.service
```

The gateway handles `help`, `status`, `stream`, `stream status`, `stream start`,
`stream stop`, and `diagnostics`. Motion, navigation, and arbitrary shell
commands are not executed.

## ROS API

- `/scout/slack/send_message`: `scoutmini_interfaces/srv/SendSlackMessage`
- `/scout/slack/incoming_message`: JSON in `std_msgs/msg/String`
- `/scout/slack/command_request`: unhandled request JSON in `std_msgs/msg/String`

Incoming JSON includes `source`, `channel`, `channel_type`, `user`, `text`,
`ts`, `thread_ts`, `event_type`, and `handled`. Outgoing service calls are
limited globally and per Slack channel.
