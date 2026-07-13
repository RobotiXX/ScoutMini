#!/usr/bin/env python3
"""Slack command responder for Scout robot status and ZED stream control."""

from __future__ import annotations

import os
import re
import subprocess
import time
from typing import Any, Dict, Optional

from scoutmini_slack.slack_status_poster import collect_status


STREAM_CONTROL_SCRIPT = os.environ.get(
    "SCOUT_STREAM_CONTROL_SCRIPT",
    "/home/nvidia/repos/ScoutMini/ros2_ws/src/scoutmini_slack/scripts/control_zed_stream.sh",
)
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")

HELP_TEXT = (
    "*Scout Slack commands*\n"
    "- `status`: show robot IPs, Tailscale IP, stream state, and stream ports\n"
    "- `stream`: show the current ZED WebRTC viewer URL\n"
    "- `stream start`: start the ZED RTSP/WebRTC stream\n"
    "- `stream stop`: stop the ZED RTSP/WebRTC stream\n"
    "- `diagnostics`: show stream and robot communication diagnostics\n"
    "- `help`: show this command list\n"
    "- Motion/navigation commands are disabled"
)


def _clean_command(text: str) -> str:
    text = re.sub(r"<@[^>]+>", "", text)
    return " ".join(text.strip().lower().split())


def _stream_text() -> str:
    status = collect_status()
    state = "online" if status.stream_online else "offline"
    return (
        "*Scout ZED stream*\n"
        f"- State: *{state}*\n"
        f"- URL: {status.stream_url}\n"
        "- Motion/navigation commands: disabled"
    )


def _run_stream_control(action: str, timeout_sec: float = 95.0) -> str:
    if not os.path.exists(STREAM_CONTROL_SCRIPT):
        return f"Missing stream control script: `{STREAM_CONTROL_SCRIPT}`"

    try:
        completed = subprocess.run(
            [STREAM_CONTROL_SCRIPT, action],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired as exc:
        output = _format_command_output(exc.stdout or "")
        return (
            f"`{action}` timed out after {timeout_sec:.0f}s.\n"
            f"Partial output:\n```{output}```"
        )
    except Exception as exc:
        return f"Failed to run stream `{action}`: `{exc}`"

    output = _format_command_output(completed.stdout or f"exit={completed.returncode}")
    prefix = f"*ZED stream {action}*"
    if completed.returncode != 0:
        prefix += f" failed with exit code {completed.returncode}"
    return f"{prefix}\n```{output}```"


def _format_command_output(output: str, max_chars: int = 3000) -> str:
    cleaned = ANSI_ESCAPE_RE.sub("", output).strip()
    if not cleaned:
        cleaned = "no output"
    if len(cleaned) <= max_chars:
        return cleaned
    return cleaned[:max_chars] + "\n... truncated ..."


def _response_for(text: str) -> str:
    command = _clean_command(text)

    if command in ("status", "scout status"):
        return collect_status().to_slack_text()
    if command in ("stream", "zed stream", "stream link", "scout stream"):
        return _stream_text()
    if command in ("stream status", "zed status", "stream check"):
        return _run_stream_control("status", timeout_sec=10.0)
    if command in ("stream start", "start stream", "zed start", "start zed stream"):
        return _run_stream_control("start")
    if command in ("stream stop", "stop stream", "zed stop", "stop zed stream"):
        return _run_stream_control("stop", timeout_sec=30.0)
    if command in ("diagnostics", "diagnostic", "diag", "stream diagnostics"):
        return _run_stream_control("diagnostics", timeout_sec=30.0)
    if command in ("help", "commands", ""):
        return HELP_TEXT

    return (
        "Unknown Scout command. Try `status`, `stream`, `stream start`, "
        "`stream stop`, `diagnostics`, or `help`.\n"
        "Motion/navigation commands are disabled."
    )


def _env(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise SystemExit(f"Missing {name} environment variable.")
    return value


def _event_channel(event: Dict[str, Any]) -> Optional[str]:
    channel = event.get("channel")
    return channel if isinstance(channel, str) and channel else None


def _event_text(event: Dict[str, Any]) -> str:
    text = event.get("text")
    return text if isinstance(text, str) else ""


def run_bot() -> None:
    bot_token = _env("SLACK_BOT_TOKEN")
    app_token = _env("SLACK_APP_TOKEN")

    try:
        from slack_sdk import WebClient
        from slack_sdk.errors import SlackApiError
        from slack_sdk.socket_mode import SocketModeClient
        from slack_sdk.socket_mode.request import SocketModeRequest
        from slack_sdk.socket_mode.response import SocketModeResponse
    except ImportError as exc:
        raise SystemExit(
            "Missing Python package slack_sdk. Run "
            "`python3 -m pip install slack_sdk` in the active environment."
        ) from exc

    web_client = WebClient(token=bot_token)
    socket_client = SocketModeClient(app_token=app_token, web_client=web_client)

    try:
        auth = web_client.auth_test()
        bot_user_id = auth.get("user_id")
    except SlackApiError:
        bot_user_id = None

    def process(client: SocketModeClient, request: SocketModeRequest) -> None:
        client.send_socket_mode_response(
            SocketModeResponse(envelope_id=request.envelope_id)
        )

        if request.type != "events_api":
            return

        event = request.payload.get("event", {})
        if not isinstance(event, dict):
            return

        if event.get("bot_id") or event.get("user") == bot_user_id:
            return

        event_type = event.get("type")
        channel_type = event.get("channel_type")
        is_supported_event = event_type == "app_mention" or (
            event_type == "message" and channel_type == "im"
        )
        if not is_supported_event:
            return

        channel = _event_channel(event)
        if not channel:
            return

        thread_ts = event.get("thread_ts") or event.get("ts")
        response = _response_for(_event_text(event))

        try:
            web_client.chat_postMessage(
                channel=channel,
                text=response,
                thread_ts=thread_ts,
            )
        except SlackApiError as exc:
            print(f"Slack API error: {exc.response.get('error')}")

    socket_client.socket_mode_request_listeners.append(process)
    try:
        socket_client.connect()
    except SlackApiError as exc:
        error = exc.response.get("error")
        if error == "invalid_auth":
            raise SystemExit(
                "Slack rejected SLACK_APP_TOKEN while opening Socket Mode. "
                "Use an app-level token that starts with `xapp-`, belongs to "
                "the same Slack app as SLACK_BOT_TOKEN, has the "
                "`connections:write` scope, and has Socket Mode enabled."
            ) from exc
        raise
    print(
        "Scout Slack command bot connected. Commands: status, stream, "
        "stream start, stream stop, diagnostics, help."
    )

    while True:
        time.sleep(60)


def main() -> None:
    run_bot()


if __name__ == "__main__":
    main()
