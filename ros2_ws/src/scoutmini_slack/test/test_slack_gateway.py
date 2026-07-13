"""Unit tests for Slack command parsing and ROS event metadata."""

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
