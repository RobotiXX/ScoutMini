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


def test_rtsp_ready_tolerates_jetson_egl_diagnostic(monkeypatch):
    monkeypatch.setattr(
        slack_status,
        "_run",
        lambda *args, **kwargs: (
            "nvbufsurftransform: Could not get EGL display connection\nh264"
        ),
    )
    assert slack_status._rtsp_ready("rtsp://127.0.0.1:8554/zed")


def test_rtsp_ready_rejects_non_video_response(monkeypatch):
    monkeypatch.setattr(slack_status, "_run", lambda *args, **kwargs: "exit=1")
    assert not slack_status._rtsp_ready("rtsp://127.0.0.1:8554/zed")


def test_collect_status_checks_local_webrtc_but_advertises_remote(monkeypatch):
    checked_urls = []
    monkeypatch.setenv("SCOUT_STREAM_URL", "http://100.64.0.1:8889/zed/")
    monkeypatch.setattr(slack_status, "_rtsp_ready", lambda url: True)
    monkeypatch.setattr(
        slack_status,
        "_webrtc_ready",
        lambda url: checked_urls.append(url) or True,
    )
    monkeypatch.setattr(slack_status, "_run", lambda *args, **kwargs: "none")

    status = slack_status.collect_status()

    assert status.stream_online
    assert status.stream_url == "http://100.64.0.1:8889/zed/"
    assert checked_urls == ["http://127.0.0.1:8889/zed/"]
