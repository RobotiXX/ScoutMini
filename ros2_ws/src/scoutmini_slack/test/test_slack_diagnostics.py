"""Tests for redacted Slack diagnostics."""

from types import SimpleNamespace

from scoutmini_slack import slack_diagnostics


def test_configuration_status_accepts_expected_shapes(monkeypatch):
    monkeypatch.setenv("SLACK_BOT_TOKEN", "xoxb-secret")
    monkeypatch.setenv("SLACK_APP_TOKEN", "xapp-secret")
    monkeypatch.setenv("SLACK_CHANNEL_ID", "C123")

    ok, message = slack_diagnostics._configuration_status()

    assert ok
    assert "secret" not in message
    assert "redacted" in message


def test_configuration_status_names_missing_values(monkeypatch):
    monkeypatch.delenv("SLACK_BOT_TOKEN", raising=False)
    monkeypatch.delenv("SLACK_APP_TOKEN", raising=False)
    monkeypatch.delenv("SLACK_CHANNEL_ID", raising=False)

    ok, message = slack_diagnostics._configuration_status()

    assert not ok
    assert "SLACK_BOT_TOKEN" in message
    assert "SLACK_APP_TOKEN" in message
    assert "SLACK_CHANNEL_ID" in message


def test_socket_mode_requires_active_service(monkeypatch):
    monkeypatch.setattr(
        slack_diagnostics.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=3,
            stdout="inactive\n",
        ),
    )

    assert slack_diagnostics._socket_mode_status() == (
        False,
        "scoutmini-slack.service is inactive",
    )


def test_socket_mode_requires_fresh_connected_state(monkeypatch, tmp_path):
    monkeypatch.setattr(
        slack_diagnostics.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0,
            stdout="active\n",
        ),
    )
    state_path = tmp_path / "slack_socket_state"
    state_path.write_text("connected\n", encoding="utf-8")
    monkeypatch.setattr(
        slack_diagnostics,
        "_socket_state_path",
        lambda: state_path,
    )

    assert slack_diagnostics._socket_mode_status() == (
        True,
        "Slack Socket Mode connection is live",
    )


def test_socket_mode_rejects_disconnected_state(monkeypatch, tmp_path):
    monkeypatch.setattr(
        slack_diagnostics.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0,
            stdout="active\n",
        ),
    )
    state_path = tmp_path / "slack_socket_state"
    state_path.write_text("disconnected\n", encoding="utf-8")
    monkeypatch.setattr(
        slack_diagnostics,
        "_socket_state_path",
        lambda: state_path,
    )

    assert slack_diagnostics._socket_mode_status() == (
        False,
        "Socket Mode state is disconnected",
    )
