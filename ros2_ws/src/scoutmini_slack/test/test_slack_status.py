"""Unit tests for stream status health semantics."""

from scoutmini_slack import slack_status


def test_webrtc_ready_accepts_successful_http(monkeypatch):
    monkeypatch.setattr(slack_status, "_run", lambda *args, **kwargs: "200")
    assert slack_status._webrtc_ready("http://127.0.0.1:8889/zed/")


def test_webrtc_ready_rejects_open_port_without_http(monkeypatch):
    monkeypatch.setattr(slack_status, "_run", lambda *args, **kwargs: "exit=7")
    assert not slack_status._webrtc_ready("http://127.0.0.1:8889/zed/")


def test_rtsp_ready_requires_h264_video(monkeypatch):
    monkeypatch.setattr(slack_status, "_run", lambda *args, **kwargs: "h264")
    assert slack_status._rtsp_ready("rtsp://127.0.0.1:8554/zed")


def test_rtsp_ready_rejects_non_video_response(monkeypatch):
    monkeypatch.setattr(slack_status, "_run", lambda *args, **kwargs: "exit=1")
    assert not slack_status._rtsp_ready("rtsp://127.0.0.1:8554/zed")
