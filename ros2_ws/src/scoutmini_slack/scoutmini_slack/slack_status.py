#!/usr/bin/env python3
"""Scout status helpers used by the Slack gateway."""

from __future__ import annotations

import os
import socket
import subprocess
from dataclasses import dataclass
from typing import Sequence


START_STREAM_COMMAND = "stream start"
RTSP_URL = os.environ.get("SCOUT_RTSP_URL", "rtsp://127.0.0.1:8554/zed")
WEBRTC_HEALTH_URL = os.environ.get(
    "SCOUT_WEBRTC_HEALTH_URL",
    "http://127.0.0.1:8889/zed/",
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


def _webrtc_ready(url: str) -> bool:
    result = _run(
        [
            "curl",
            "--fail",
            "--silent",
            "--output",
            "/dev/null",
            "--write-out",
            "%{http_code}",
            "--max-time",
            "3",
            url,
        ],
        timeout_sec=4.0,
    )
    return result.startswith("2") or result.startswith("3")


def _rtsp_ready(url: str) -> bool:
    result = _run(
        [
            "ffprobe",
            "-v",
            "error",
            "-rtsp_transport",
            "tcp",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=codec_name",
            "-of",
            "default=noprint_wrappers=1:nokey=1",
            url,
        ],
        timeout_sec=8.0,
    )
    return "h264" in result.splitlines()


def collect_status() -> ScoutStatus:
    hostname = socket.gethostname()
    ips = _run(["hostname", "-I"])
    tailscale_ip = _run(["tailscale", "ip", "-4"])
    stream_host = (
        tailscale_ip.split()[0]
        if tailscale_ip and not tailscale_ip.startswith("unavailable:")
        else "<robot_ip_or_tailscale_ip>"
    )
    stream_url = os.environ.get(
        "SCOUT_STREAM_URL",
        f"http://{stream_host}:8889/zed/",
    )
    stream_ports = _run(
        [
            "bash",
            "-lc",
            "ss -ltnup | grep -E ':8554|:8889|:8189' || echo 'none'",
        ]
    )
    return ScoutStatus(
        hostname=hostname,
        ips=ips,
        tailscale_ip=tailscale_ip,
        stream_url=stream_url,
        stream_online=(
            _rtsp_ready(RTSP_URL) and _webrtc_ready(WEBRTC_HEALTH_URL)
        ),
        stream_ports=stream_ports,
    )
