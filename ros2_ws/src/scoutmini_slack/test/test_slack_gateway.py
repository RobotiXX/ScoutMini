"""Unit tests for Slack command parsing and ROS event metadata."""

from types import SimpleNamespace
from unittest.mock import Mock

from scoutmini_slack.slack_gateway import SlackGateway
from scoutmini_slack.slack_gateway import TokenBucket
from scoutmini_slack.slack_gateway import _clean_command
from scoutmini_slack.slack_gateway import _stream_control_script


def test_clean_command_removes_mention_and_normalizes_whitespace():
    assert _clean_command(" <@U123>  Stream   START ") == "stream start"


def test_incoming_payload_keeps_routing_metadata():
    event = {
        "channel": "C123",
        "channel_type": "channel",
        "user": "U123",
        "ts": "100.1",
        "thread_ts": "99.9",
        "type": "app_mention",
    }

    payload = SlackGateway._incoming_message_payload(
        event,
        "status",
        handled=True,
    )

    assert payload == {
        "source": "slack",
        "channel": "C123",
        "channel_type": "channel",
        "user": "U123",
        "text": "status",
        "ts": "100.1",
        "thread_ts": "99.9",
        "event_type": "app_mention",
        "handled": True,
    }


def test_top_level_message_does_not_invent_thread_timestamp():
    payload = SlackGateway._incoming_message_payload(
        {
            "channel": "C123",
            "user": "U123",
            "ts": "100.1",
            "type": "app_mention",
        },
        "status",
        handled=True,
    )

    assert payload["ts"] == "100.1"
    assert payload["thread_ts"] is None


def test_socket_request_wrapper_logs_processing_exception():
    gateway = SlackGateway.__new__(SlackGateway)
    gateway._process_socket_request = Mock(side_effect=RuntimeError("boom"))
    logger = Mock()
    gateway.get_logger = Mock(return_value=logger)

    gateway._process_socket_request_safely(Mock())

    logger.error.assert_called_once()
    assert "boom" in logger.error.call_args.args[0]


def test_stream_control_override_is_respected(monkeypatch):
    monkeypatch.setenv("SCOUT_STREAM_CONTROL_SCRIPT", "/tmp/control-stream")
    assert _stream_control_script() == "/tmp/control-stream"


def test_token_bucket_refills_without_exceeding_burst(monkeypatch):
    times = iter((10.0, 12.0))
    monkeypatch.setattr("time.monotonic", lambda: next(times))
    bucket = TokenBucket(rate_per_sec=1.0, burst=2)
    bucket.consume()
    bucket.consume()
    bucket.refill(12.0)
    assert bucket.tokens == 2.0


def test_socket_event_logs_failed_reply():
    gateway = SlackGateway.__new__(SlackGateway)
    gateway._bot_user_id = "UBOT"
    gateway._response_for = Mock(return_value=("reply", True))
    gateway._publish_incoming_message = Mock()
    gateway._publish_command_request = Mock()
    gateway._post_message = Mock(return_value=(False, "channel_not_found", ""))
    logger = Mock()
    gateway.get_logger = Mock(return_value=logger)
    request = SimpleNamespace(
        type="events_api",
        payload={
            "event": {
                "type": "app_mention",
                "channel": "C123",
                "channel_type": "channel",
                "user": "U123",
                "text": "status",
                "ts": "100.1",
            }
        },
    )

    gateway._process_socket_request(request)

    gateway._post_message.assert_called_once_with(
        channel="C123",
        text="reply",
        thread_ts="100.1",
    )
    logger.error.assert_called_once()
    assert "channel_not_found" in logger.error.call_args.args[0]
