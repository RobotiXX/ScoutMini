#!/usr/bin/env python3
"""Post Scout robot status to Slack without exposing robot control."""

from __future__ import annotations

import os
from scoutmini_slack.slack_status import collect_status


def post_status() -> None:
    token = os.environ.get("SLACK_BOT_TOKEN", "").strip()
    channel = os.environ.get("SLACK_CHANNEL_ID", "").strip()

    if not token:
        raise SystemExit("Missing SLACK_BOT_TOKEN environment variable.")
    if not channel:
        raise SystemExit("Missing SLACK_CHANNEL_ID environment variable.")

    try:
        from slack_sdk import WebClient
        from slack_sdk.errors import SlackApiError
    except ImportError as exc:
        raise SystemExit(
            "Missing Python package slack_sdk. Run "
            "`python3 -m pip install slack_sdk` in the active environment."
        ) from exc

    client = WebClient(token=token)
    status = collect_status()

    try:
        client.chat_postMessage(channel=channel, text=status.to_slack_text())
    except SlackApiError as exc:
        error = exc.response.get("error")
        raise SystemExit(f"Slack API error: {error}") from exc

    print(f"Posted Scout status to Slack channel {channel}.")


def main() -> None:
    post_status()


if __name__ == "__main__":
    main()
