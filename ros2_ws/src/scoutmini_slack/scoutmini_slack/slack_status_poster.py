#!/usr/bin/env python3
"""Post Scout robot status to Slack without exposing robot control."""

from __future__ import annotations

import os
import socket
import subprocess
from dataclasses import dataclass
from typing import List


WEBRTC_PORT = 8889
START_STREAM_COMMAND = (
    "cd /home/nvidia/repos/ScoutMini && "
    "ros2_ws/src/scoutmini_slack/scripts/control_zed_stream.sh start"
)


def _run(command: List[str], timeout_sec: float = 5.0) -> str:
    try:
        completed = subprocess.run(
            command,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec,
        )
    except Exception as exc:
        return f"unavailable: {exc}"

    output = completed.stdout.strip()
    return output if output else f"exit={completed.returncode}"


@dataclass
class ScoutStatus:
    hostname: str
    ips: str
    tailscale_ip: str
    stream_url: str
    stream_online: bool
    stream_ports: str

    def to_slack_text(self) -> str:
        stream_state = "online" if self.stream_online else "offline"
        stream_line = f"- ZED stream: *{stream_state}*"
        if self.stream_online:
            stream_line += f" | {self.stream_url}"
        else:
            stream_line += f"\n- Start stream: `{START_STREAM_COMMAND}`"

        return (
            "*Scout status*\n"
            f"- Host: `{self.hostname}`\n"
            f"- IPs: `{self.ips}`\n"
            f"- Tailscale: `{self.tailscale_ip}`\n"
            f"{stream_line}\n"
            f"- Stream ports: ```{self.stream_ports}```\n"
            "- Motion commands: disabled for this Slack prototype"
        )


def _is_port_listening(port: int) -> bool:
    result = _run(["bash", "-lc", f"ss -ltn | awk '{{print $4}}' | grep -Eq ':{port}$'"])
    return result.startswith("exit=0")


def collect_status() -> ScoutStatus:
    hostname = socket.gethostname()
    ips = _run(["hostname", "-I"])
    tailscale_ip = _run(["tailscale", "ip", "-4"])
    stream_host = (
        tailscale_ip.split()[0]
        if tailscale_ip and not tailscale_ip.startswith("unavailable:")
        else "<robot_ip_or_tailscale_ip>"
    )
    stream_url = os.environ.get("SCOUT_STREAM_URL", f"http://{stream_host}:8889/zed/")
    stream_ports = _run(
        ["bash", "-lc", "ss -ltnup | grep -E ':8554|:8889|:8189' || echo 'none'"]
    )
    return ScoutStatus(
        hostname=hostname,
        ips=ips,
        tailscale_ip=tailscale_ip,
        stream_url=stream_url,
        stream_online=_is_port_listening(WEBRTC_PORT),
        stream_ports=stream_ports,
    )


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
        raise SystemExit(f"Slack API error: {exc.response.get('error')}") from exc

    print(f"Posted Scout status to Slack channel {channel}.")


def main() -> None:
    post_status()


if __name__ == "__main__":
    main()
