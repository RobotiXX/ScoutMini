# Slack ROS Gateway Handoff

Date: 2026-07-08

## Short Summary

We implemented and tested a Slack-to-ROS gateway in a clean worktree, separate
from the dirty main ScoutMini checkout. Slack can now send messages into ROS,
and ROS nodes can send Slack messages through a ROS service. The ROS send
service now includes a local rate guard to prevent accidental Slack spam.

## Why a Separate Worktree Was Used

The main checkout at `/home/nvidia/repos/ScoutMini` already has unrelated local
changes in navigation, maps, route runner files, submodules, `.gitignore`, and
Slack prototype files. The Slack ROS gateway also touches some overlapping files,
especially:

- `.gitignore`
- `ros2_ws/src/scoutmini_tasks/setup.py`
- `docs/remote_communication/`
- `scripts/slack/`

To avoid mixing PR work with local WIP, the gateway was recovered and tested in:

```text
/home/nvidia/repos/ScoutMini-slack-pr2
```

That directory is a Git worktree for the same repository, not an independent
clone.

## Implemented Work

New ROS interface package:

```text
ros2_ws/src/scoutmini_interfaces/
```

New service:

```text
scoutmini_interfaces/srv/SendSlackMessage
```

Request fields:

```text
string channel
string text
string thread_ts
```

Response fields:

```text
bool success
string error
string ts
```

New Slack gateway node:

```text
ros2_ws/src/scoutmini_tasks/scoutmini_tasks/slack_gateway.py
```

ROS API:

```text
Node:    /slack_gateway
Service: /scout/slack/send_message
Topic:   /scout/slack/command_request
```

The command-request topic publishes JSON as `std_msgs/msg/String` for
unrecognized Slack requests. This creates a future extension point without
letting Slack directly run navigation, motion, `cmd_vel`, route execution, or
arbitrary shell commands.

## Slack Commands Preserved

```text
help
status
stream
stream status
stream start
stream stop
diagnostics
```

The gateway uses:

- `SLACK_BOT_TOKEN` for Slack Web API posting.
- `SLACK_APP_TOKEN` for Slack Socket Mode.
- `.local/secrets/slack.env` for local secrets.

No real Slack tokens were committed.

## Send Message Rate Guard

The `/scout/slack/send_message` service has an in-memory token-bucket limiter.
It is intended to stop a buggy ROS node from flooding Slack.

Default limits:

```text
Per channel: 1 message every 2 seconds, burst 3
Global:      1 message every 1 second, burst 10
```

If a service call is denied by the limiter, the response is:

```text
success=false
error='rate_limited'
ts=''
```

The gateway logs the denied channel and retry window. Messages are not queued;
callers should retry later if the message still matters.

## Test Commands

From the working gateway worktree:

```bash
cd /home/nvidia/repos/ScoutMini-slack-pr2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scoutmini_interfaces scoutmini_tasks
source install/setup.bash
source /home/nvidia/repos/ScoutMini/.local/secrets/slack.env
ros2 run scoutmini_tasks slack_gateway
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/repos/ScoutMini-slack-pr2/ros2_ws/install/setup.bash
source /home/nvidia/repos/ScoutMini/.local/secrets/slack.env

ros2 node info /slack_gateway
ros2 service type /scout/slack/send_message
ros2 topic list | grep /scout/slack/command_request
```

Send a Slack message through ROS:

```bash
ros2 service call /scout/slack/send_message scoutmini_interfaces/srv/SendSlackMessage "{channel: ${SLACK_CHANNEL_ID}, text: 'ScoutMini Slack gateway test', thread_ts: ''}"
```

Expected result:

```text
success=True
error=''
ts='<slack timestamp>'
```

Rate-limit smoke test:

```bash
for i in 1 2 3 4 5; do
  ros2 service call /scout/slack/send_message scoutmini_interfaces/srv/SendSlackMessage "{channel: ${SLACK_CHANNEL_ID}, text: 'ScoutMini rate-limit test '$i, thread_ts: ''}" &
done
wait
```

Expected result: the first same-channel burst allows up to three messages, and
excess immediate calls return `error='rate_limited'`. After about two seconds,
the next same-channel message should be allowed.

Watch unrecognized Slack commands:

```bash
ros2 topic echo /scout/slack/command_request
```

Then DM the bot an unknown command, such as `go to lobby`. The gateway should
not move the robot. It should publish JSON on the ROS topic.

## Validation Completed

The following checks passed in the worktree:

- Python parse check for Slack gateway files.
- `bash -n` for Slack shell scripts.
- `git diff --check`.
- `colcon build --symlink-install --packages-select scoutmini_interfaces scoutmini_tasks`.
- Gateway started and connected to Slack Socket Mode.
- `/scout/slack/send_message` appeared in ROS.
- `/scout/slack/command_request` appeared in ROS.
- A ROS service call successfully posted to Slack and returned a Slack timestamp.
- Burst testing confirmed same-channel service calls are rate-limited after the
  configured burst.
- Refill testing confirmed a later service call succeeds after the two-second
  channel refill window.

## Not Yet Done

- No PR was opened.
- No commit was created.
- `stream start` and `stream stop` were not exercised during final gateway
  testing because they should only be run when the robot is stationary and it is
  safe to start or stop the stream stack.
- The work has not been merged into the dirty main checkout.

## Cleanup Note

A recovery backup may still exist at:

```text
/home/nvidia/ScoutMini-slack-pr2-recovered
```

After the worktree is committed or otherwise preserved, that backup can be
removed.
