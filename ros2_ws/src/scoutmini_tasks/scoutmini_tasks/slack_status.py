#!/usr/bin/env python3
"""Scout status helpers used by the Slack gateway."""

from __future__ import annotations

import os
import socket
import subprocess
from dataclasses import dataclass
from typing import Sequence


WEBRTC_PORT = 8889
START_STREAM_COMMAND = (
    "cd /home/nvidia/repos/ScoutMini && "
    "./scripts/slack/control_zed_stream.sh start"
)


def _run(command: Sequence[str], timeout_sec: float = 5.0) -> str:
    try:
        completed = subprocess.run(
            list(command),
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


@dataclass(frozen=True)
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
            "- Motion commands: disabled"
        )


def _is_port_listening(port: int) -> bool:
    result = _run(["bash", "-lc", f"ss -ltn | awk '{{print $4}}' | grep -Eq ':{port}$'"])
    return result.startswith("exit=0")


def collect_status() -> ScoutStatus:
    stream_url = os.environ.get("SCOUT_STREAM_URL", "http://100.78.242.13:8889/zed/")
    stream_ports = _run(
        ["bash", "-lc", "ss -ltnup | grep -E ':8554|:8889|:8189' || echo 'none'"]
    )
    return ScoutStatus(
        hostname=socket.gethostname(),
        ips=_run(["hostname", "-I"]),
        tailscale_ip=_run(["tailscale", "ip", "-4"]),
        stream_url=stream_url,
        stream_online=_is_port_listening(WEBRTC_PORT),
        stream_ports=stream_ports,
    )
