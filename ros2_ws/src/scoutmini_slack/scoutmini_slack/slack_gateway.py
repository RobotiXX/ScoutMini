#!/usr/bin/env python3
"""ROS-native Slack gateway for ScoutMini."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time
from typing import Any, Dict, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from scoutmini_interfaces.srv import SendSlackMessage
from std_msgs.msg import String

from scoutmini_slack.slack_status import collect_status


ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
SOCKET_STATE_INTERVAL_SEC = 2.0

HELP_TEXT = (
    "*Scout Slack commands*\n"
    "- `status`: show IPs, Tailscale, stream state, and stream ports\n"
    "- `stream`: show the current ZED WebRTC viewer URL\n"
    "- `stream status`: show ZED stream process and port status\n"
    "- `stream start`: start the WebRTC gateway after camera/RTSP bringup\n"
    "- `stream stop`: stop the ZED RTSP/WebRTC stream\n"
    "- `diagnostics`: show stream and robot communication diagnostics\n"
    "- `help`: show this command list\n"
    "- Motion/navigation commands are disabled"
)


def _clean_command(text: str) -> str:
    text = re.sub(r"<@[^>]+>", "", text)
    return " ".join(text.strip().lower().split())


def _format_command_output(output: str, max_chars: int = 3000) -> str:
    cleaned = ANSI_ESCAPE_RE.sub("", output).strip()
    if not cleaned:
        cleaned = "no output"
    if len(cleaned) <= max_chars:
        return cleaned
    return cleaned[:max_chars] + "\n... truncated ..."


def _stream_control_script() -> str:
    override = os.environ.get("SCOUT_STREAM_CONTROL_SCRIPT", "").strip()
    if override:
        return override
    return os.path.join(
        get_package_share_directory("scoutmini_slack"),
        "scripts",
        "control_zed_stream.sh",
    )


class TokenBucket:
    def __init__(self, rate_per_sec: float, burst: int) -> None:
        self.rate_per_sec = max(rate_per_sec, 0.0)
        self.burst = max(burst, 1)
        self.tokens = float(self.burst)
        self.updated_at = time.monotonic()

    def refill(self, now: float) -> None:
        elapsed = max(now - self.updated_at, 0.0)
        self.tokens = min(
            self.burst,
            self.tokens + elapsed * self.rate_per_sec,
        )
        self.updated_at = now

    def can_consume(self) -> bool:
        return self.tokens >= 1.0

    def consume(self) -> None:
        self.tokens -= 1.0

    def retry_after_sec(self) -> float:
        if self.can_consume() or self.rate_per_sec <= 0.0:
            return 0.0
        return (1.0 - self.tokens) / self.rate_per_sec


class SlackGateway(Node):
    def __init__(self) -> None:
        super().__init__("slack_gateway")
        self.declare_parameter("send_global_rate_per_sec", 1.0)
        self.declare_parameter("send_global_burst", 10)
        self.declare_parameter("send_channel_rate_per_sec", 0.5)
        self.declare_parameter("send_channel_burst", 3)
        self._command_pub = self.create_publisher(
            String, "/scout/slack/command_request", 10
        )
        self._incoming_pub = self.create_publisher(
            String, "/scout/slack/incoming_message", 10
        )
        self._service = self.create_service(
            SendSlackMessage,
            "/scout/slack/send_message",
            self._handle_send_message,
        )
        self._slack_lock = threading.Lock()
        self._rate_limit_lock = threading.Lock()
        self._request_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="slack-request",
        )
        self._global_send_bucket = TokenBucket(
            self.get_parameter("send_global_rate_per_sec")
            .get_parameter_value()
            .double_value,
            self.get_parameter("send_global_burst")
            .get_parameter_value()
            .integer_value,
        )
        self._channel_send_buckets: Dict[str, TokenBucket] = {}
        self._channel_send_rate_per_sec = (
            self.get_parameter("send_channel_rate_per_sec")
            .get_parameter_value()
            .double_value
        )
        self._channel_send_burst = (
            self.get_parameter("send_channel_burst")
            .get_parameter_value()
            .integer_value
        )
        self._web_client: Any = None
        self._socket_client: Any = None
        self._bot_user_id: Optional[str] = None
        self._slack_errors: Tuple[Any, ...] = ()
        self._socket_state_path = self._runtime_state_path()
        self._socket_state_timer = self.create_timer(
            SOCKET_STATE_INTERVAL_SEC,
            self._write_socket_state,
        )

        self._init_slack()

    def _init_slack(self) -> None:
        bot_token = self._required_env("SLACK_BOT_TOKEN")
        app_token = self._required_env("SLACK_APP_TOKEN")

        try:
            from slack_sdk import WebClient
            from slack_sdk.errors import SlackApiError
            from slack_sdk.socket_mode import SocketModeClient
        except ImportError as exc:
            raise SystemExit(
                "Missing Python package slack_sdk. Install it in the active "
                "ROS environment, for example with "
                "`python3 -m pip install slack_sdk`."
            ) from exc

        self._slack_errors = (SlackApiError,)
        self._web_client = WebClient(token=bot_token)
        self._socket_client = SocketModeClient(
            app_token=app_token,
            web_client=self._web_client,
        )

        try:
            auth = self._web_client.auth_test()
            user_id = auth.get("user_id")
            self._bot_user_id = user_id if isinstance(user_id, str) else None
        except SlackApiError as exc:
            raise SystemExit(
                f"Slack auth_test failed: {exc.response.get('error')}"
            ) from exc
        except Exception as exc:
            raise SystemExit(f"Slack auth_test unavailable: {exc}") from exc

        if not self._bot_user_id:
            raise SystemExit("Slack auth_test did not return a bot user ID.")

    @staticmethod
    def _required_env(name: str) -> str:
        value = os.environ.get(name, "").strip()
        if not value:
            raise SystemExit(f"Missing {name} environment variable.")
        return value

    def start_socket_mode(self) -> None:
        from slack_sdk.errors import SlackApiError
        from slack_sdk.socket_mode.response import SocketModeResponse

        def process(client: Any, request: Any) -> None:
            client.send_socket_mode_response(
                SocketModeResponse(envelope_id=request.envelope_id)
            )
            self._request_executor.submit(
                self._process_socket_request_safely,
                request,
            )

        self._socket_client.socket_mode_request_listeners.append(process)
        try:
            self._socket_client.connect()
        except SlackApiError as exc:
            error = exc.response.get("error")
            if error == "invalid_auth":
                raise SystemExit(
                    "Slack rejected SLACK_APP_TOKEN while opening Socket "
                    "Mode. "
                    "Use an xapp token with connections:write on the same app "
                    "as SLACK_BOT_TOKEN, and make sure Socket Mode is enabled."
                ) from exc
            raise
        except Exception as exc:
            raise SystemExit(
                f"Slack Socket Mode connection failed: {exc}"
            ) from exc

        self.get_logger().info(
            "Slack gateway connected. Commands: help, status, stream, "
            "stream status, stream start, stream stop, diagnostics."
        )
        self._write_socket_state()

    def _process_socket_request_safely(self, request: Any) -> None:
        try:
            self._process_socket_request(request)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to process Slack Socket Mode request: {exc}"
            )

    @staticmethod
    def _runtime_state_path() -> Path:
        runtime_root = os.environ.get("XDG_RUNTIME_DIR", "/tmp")
        return Path(runtime_root) / "scoutmini" / "slack_socket_state"

    def _write_socket_state(self) -> None:
        connected = bool(
            self._socket_client is not None
            and self._socket_client.is_connected()
        )
        self._socket_state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self._socket_state_path.with_suffix(".tmp")
        temporary_path.write_text(
            "connected\n" if connected else "disconnected\n",
            encoding="utf-8",
        )
        temporary_path.replace(self._socket_state_path)

    def close(self) -> None:
        self._request_executor.shutdown(wait=False, cancel_futures=True)
        if self._socket_client is not None:
            self._socket_client.close()
        self._socket_state_path.unlink(missing_ok=True)

    def _process_socket_request(self, request: Any) -> None:
        if request.type != "events_api":
            return

        event = request.payload.get("event", {})
        if not isinstance(event, dict):
            return

        if event.get("bot_id") or event.get("user") == self._bot_user_id:
            return

        event_type = event.get("type")
        channel_type = event.get("channel_type")
        supported = event_type == "app_mention" or (
            event_type == "message" and channel_type == "im"
        )
        if not supported:
            return

        channel = self._event_channel(event)
        if not channel:
            return

        thread_ts = self._event_thread_ts(event)
        text = self._event_text(event)
        response, handled = self._response_for(text, event)
        self._publish_incoming_message(event, text, handled)
        if not handled:
            self._publish_command_request(event, text)

        posted, error, _ = self._post_message(
            channel=channel,
            text=response,
            thread_ts=thread_ts,
        )
        if not posted:
            self.get_logger().error(
                f"Failed to reply to Slack channel {channel}: {error}"
            )

    @staticmethod
    def _event_channel(event: Dict[str, Any]) -> Optional[str]:
        channel = event.get("channel")
        return channel if isinstance(channel, str) and channel else None

    @staticmethod
    def _event_text(event: Dict[str, Any]) -> str:
        text = event.get("text")
        return text if isinstance(text, str) else ""

    @staticmethod
    def _event_thread_ts(event: Dict[str, Any]) -> Optional[str]:
        thread_ts = event.get("thread_ts") or event.get("ts")
        return thread_ts if isinstance(thread_ts, str) and thread_ts else None

    def _response_for(
        self,
        text: str,
        event: Dict[str, Any],
    ) -> Tuple[str, bool]:
        command = _clean_command(text)

        if command in ("status", "scout status"):
            return collect_status().to_slack_text(), True
        if command in ("stream", "zed stream", "stream link", "scout stream"):
            return self._stream_text(), True
        if command in ("stream status", "zed status", "stream check"):
            return self._run_stream_control("status", timeout_sec=10.0), True
        if command in (
            "stream start",
            "start stream",
            "zed start",
            "start zed stream",
        ):
            return self._run_stream_control("start"), True
        if command in (
            "stream stop",
            "stop stream",
            "zed stop",
            "stop zed stream",
        ):
            return self._run_stream_control("stop", timeout_sec=30.0), True
        if command in (
            "diagnostics",
            "diagnostic",
            "diag",
            "stream diagnostics",
        ):
            return (
                self._run_stream_control("diagnostics", timeout_sec=30.0),
                True,
            )
        if command in ("help", "commands", ""):
            return HELP_TEXT, True

        user = event.get("user", "unknown")
        return (
            "I published that request for ROS-side handling, but this "
            "gateway does not execute motion, navigation, or shell "
            "commands directly.\n"
            f"- Requester: `{user}`\n"
            "- Try `status`, `stream`, `stream status`, `stream start`, "
            "`stream stop`, `diagnostics`, or `help`.",
            False,
        )

    def _stream_text(self) -> str:
        status = collect_status()
        state = "reachable" if status.stream_online else "unreachable"
        return (
            "*Scout ZED stream*\n"
            f"- Gateway: *{state}*\n"
            f"- URL: {status.stream_url}\n"
            "- Motion/navigation commands: disabled"
        )

    def _run_stream_control(
        self,
        action: str,
        timeout_sec: float = 95.0,
    ) -> str:
        control_script = _stream_control_script()
        if not os.path.exists(control_script):
            return f"Missing stream control script: `{control_script}`"

        try:
            completed = subprocess.run(
                [control_script, action],
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

        output = _format_command_output(
            completed.stdout or f"exit={completed.returncode}"
        )
        prefix = f"*ZED stream {action}*"
        if completed.returncode != 0:
            prefix += f" failed with exit code {completed.returncode}"
        return f"{prefix}\n```{output}```"

    def _publish_command_request(
        self,
        event: Dict[str, Any],
        text: str,
    ) -> None:
        payload = self._incoming_message_payload(event, text, handled=False)
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._command_pub.publish(message)

    def _publish_incoming_message(
        self,
        event: Dict[str, Any],
        text: str,
        handled: bool,
    ) -> None:
        payload = self._incoming_message_payload(event, text, handled=handled)
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._incoming_pub.publish(message)

    @staticmethod
    def _incoming_message_payload(
        event: Dict[str, Any],
        text: str,
        *,
        handled: bool,
    ) -> Dict[str, Any]:
        return {
            "source": "slack",
            "channel": event.get("channel"),
            "channel_type": event.get("channel_type"),
            "user": event.get("user"),
            "text": text,
            "ts": event.get("ts"),
            "thread_ts": event.get("thread_ts"),
            "event_type": event.get("type"),
            "handled": handled,
        }

    def _post_message(
        self,
        *,
        channel: str,
        text: str,
        thread_ts: Optional[str] = None,
    ) -> Tuple[bool, str, str]:
        kwargs = {"channel": channel, "text": text}
        if thread_ts:
            kwargs["thread_ts"] = thread_ts

        try:
            with self._slack_lock:
                response = self._web_client.chat_postMessage(**kwargs)
        except self._slack_errors as exc:
            return False, str(exc.response.get("error")), ""
        except Exception as exc:
            return False, str(exc), ""

        ts = response.get("ts")
        return True, "", ts if isinstance(ts, str) else ""

    def _check_service_send_rate_limit(self, channel: str) -> Tuple[bool, str]:
        now = time.monotonic()

        with self._rate_limit_lock:
            channel_bucket = self._channel_send_buckets.get(channel)
            if channel_bucket is None:
                channel_bucket = TokenBucket(
                    self._channel_send_rate_per_sec,
                    self._channel_send_burst,
                )
                self._channel_send_buckets[channel] = channel_bucket

            self._global_send_bucket.refill(now)
            channel_bucket.refill(now)

            if not self._global_send_bucket.can_consume():
                retry_after = self._global_send_bucket.retry_after_sec()
                return False, f"global retry_after={retry_after:.1f}s"

            if not channel_bucket.can_consume():
                retry_after = channel_bucket.retry_after_sec()
                return (
                    False,
                    f"channel={channel} retry_after={retry_after:.1f}s",
                )

            self._global_send_bucket.consume()
            channel_bucket.consume()
            return True, ""

    def _handle_send_message(
        self,
        request: SendSlackMessage.Request,
        response: SendSlackMessage.Response,
    ) -> SendSlackMessage.Response:
        channel = request.channel.strip()
        text = request.text.strip()
        thread_ts_value = request.thread_ts.strip()
        thread_ts = (
            None
            if thread_ts_value.lower() in ("", "none", "null")
            else thread_ts_value
        )

        if not channel:
            response.success = False
            response.error = "channel is required"
            response.ts = ""
            return response
        if not text:
            response.success = False
            response.error = "text is required"
            response.ts = ""
            return response

        allowed, rate_limit_reason = self._check_service_send_rate_limit(
            channel
        )
        if not allowed:
            self.get_logger().warning(
                f"Rate-limited /scout/slack/send_message: {rate_limit_reason}"
            )
            response.success = False
            response.error = "rate_limited"
            response.ts = ""
            return response

        success, error, ts = self._post_message(
            channel=channel,
            text=text,
            thread_ts=thread_ts,
        )
        response.success = success
        response.error = error
        response.ts = ts
        return response


def main() -> None:
    rclpy.init()
    node = SlackGateway()
    node.start_socket_mode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
