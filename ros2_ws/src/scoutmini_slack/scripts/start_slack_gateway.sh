#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-$HOME/repos/ScoutMini}"
SCOUT_WS="${SCOUT_WS:-$REPO_ROOT/ros2_ws}"
SLACK_ENV="${SLACK_ENV:-$REPO_ROOT/.local/secrets/slack.env}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Missing /opt/ros/humble/setup.bash. Install ROS 2 Humble first." >&2
  exit 1
fi

if [[ ! -f "$SCOUT_WS/install/setup.bash" ]]; then
  echo "Missing $SCOUT_WS/install/setup.bash. Build the ScoutMini ROS workspace first." >&2
  exit 1
fi

if [[ ! -f "$SLACK_ENV" ]]; then
  echo "Missing Slack env file: $SLACK_ENV" >&2
  echo "Copy ros2_ws/src/scoutmini_slack/config/slack.env.example to that path and fill it in." >&2
  exit 1
fi

if [[ -n "$(find "$SLACK_ENV" -maxdepth 0 -perm /077 -print -quit)" ]]; then
  echo "Slack env file is readable or writable by group/other: $SLACK_ENV" >&2
  echo "Run: chmod 600 $SLACK_ENV" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$SCOUT_WS/install/setup.bash"
source "$SLACK_ENV"
set -u

missing=()
for name in SLACK_BOT_TOKEN SLACK_APP_TOKEN; do
  if [[ -z "${!name:-}" || "${!name}" == *your-token-here* ]]; then
    missing+=("$name")
  fi
done

if (( ${#missing[@]} > 0 )); then
  echo "Missing required Slack config in $SLACK_ENV: ${missing[*]}" >&2
  exit 1
fi

exec ros2 run scoutmini_slack slack_gateway
