#!/usr/bin/env python3
"""Redacted Slack configuration, authentication, and lifecycle diagnostics."""

from __future__ import annotations

import os
import subprocess
from typing import Tuple


def _configuration_status() -> Tuple[bool, str]:
    bot_token = os.environ.get("SLACK_BOT_TOKEN", "").strip()
    app_token = os.environ.get("SLACK_APP_TOKEN", "").strip()
    channel = os.environ.get("SLACK_CHANNEL_ID", "").strip()
    missing = []
    if not bot_token.startswith("xoxb-"):
        missing.append("SLACK_BOT_TOKEN")
    if not app_token.startswith("xapp-"):
        missing.append("SLACK_APP_TOKEN")
    if not channel:
        missing.append("SLACK_CHANNEL_ID")
    if missing:
        return False, "missing or invalid: " + ", ".join(missing)
    return True, "required Slack values are present (values redacted)"


def _bot_auth_status() -> Tuple[bool, str]:
    try:
        from slack_sdk import WebClient
        from slack_sdk.errors import SlackApiError
    except ImportError:
        return False, "python package slack_sdk is missing"

    try:
        response = WebClient(
            token=os.environ.get("SLACK_BOT_TOKEN", "")
        ).auth_test()
    except SlackApiError as exc:
        return False, str(exc.response.get("error", "Slack API error"))
    except Exception as exc:
        return False, f"connection failed: {exc}"

    if response.get("ok") and response.get("user_id"):
        return True, "Slack bot token authenticated"
    return False, "Slack auth.test returned an incomplete response"


def _socket_mode_status() -> Tuple[bool, str]:
    try:
        completed = subprocess.run(
            ["systemctl", "--user", "is-active", "scoutmini-slack.service"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except Exception as exc:
        return False, f"service state unavailable: {exc}"

    state = completed.stdout.strip() or "unknown"
    if completed.returncode == 0 and state == "active":
        return True, "scoutmini-slack.service is active"
    return False, f"scoutmini-slack.service is {state}"


def _report(stage: str, result: Tuple[bool, str]) -> bool:
    ok, message = result
    print(f"{'PASS' if ok else 'FAIL'} {stage:<18} {message}")
    return ok


def main() -> None:
    configured = _report("CONFIG", _configuration_status())
    authenticated = _report(
        "SLACK_AUTH",
        _bot_auth_status() if configured else (False, "configuration failed"),
    )
    socket_mode = _report("SOCKET_MODE", _socket_mode_status())
    raise SystemExit(0 if configured and authenticated and socket_mode else 1)


if __name__ == "__main__":
    main()
