# Slack And Zenoh Message Contract

Slack is a human-facing request and status channel. It does not expose direct
robot motion, Nav2 actions, `cmd_vel`, or arbitrary shell execution.

## Slack Gateway

The `slack_gateway` node is the only long-running Socket Mode client. It handles
status, help, diagnostics, and WebRTC service commands. Every supported event
is published to `/scout/slack/incoming_message`. Unrecognized commands are also
published to `/scout/slack/command_request` for a separate ROS-side validator.

Payloads use JSON with these fields:

```json
{
  "source": "slack",
  "channel": "C...",
  "channel_type": "channel",
  "user": "U...",
  "text": "message text",
  "ts": "message timestamp",
  "thread_ts": null,
  "event_type": "app_mention",
  "handled": true
}
```

`thread_ts` is `null` for a top-level message and contains Slack's original
thread timestamp for a reply.

Run the installed, manually managed service with:

```bash
systemctl --user start scoutmini-slack.service
```

## Zenoh Boundary

Only low-rate structured state and validated task requests should cross Zenoh:

```text
scout/status/heartbeat
scout/task/request
scout/task/status
scout/help/request
```

Images, depth, point clouds, and other high-rate topics are not bridged by
default. Robot-side validation remains required before any accepted request can
cause a ROS action.
