"""Process-level tests for staged streaming health checks."""

import os
from pathlib import Path
import stat
import subprocess
import time


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
HEALTH_SCRIPT = PACKAGE_ROOT / "scripts" / "stream_health.sh"
CHECK_SCRIPT = PACKAGE_ROOT / "scripts" / "check_zed_stream_stack.sh"
STACK_SCRIPT = PACKAGE_ROOT / "scripts" / "start_zed_webrtc_stack.sh"


def _write_executable(path: Path, contents: str) -> Path:
    path.write_text(contents, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


def _fake_environment(tmp_path: Path) -> dict[str, str]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    workspace = tmp_path / "ws"
    install = workspace / "install"
    install.mkdir(parents=True)
    (install / "setup.bash").write_text(
        f'export PATH="{bin_dir}:$PATH"\n',
        encoding="utf-8",
    )
    _write_executable(
        bin_dir / "ros2",
        '#!/usr/bin/env bash\nexit "${FAKE_ROS_EXIT:-0}"\n',
    )
    _write_executable(
        bin_dir / "ffprobe",
        '#!/usr/bin/env bash\n'
        '[[ "${FAKE_EGL_WARNING:-0}" == 1 ]] && '
        "echo 'nvbufsurftransform: Could not get EGL display connection'\n"
        'printf "%s\\n" "${FAKE_CODEC:-h264}"\n'
        'exit "${FAKE_FFPROBE_EXIT:-0}"\n',
    )
    _write_executable(
        bin_dir / "curl",
        '#!/usr/bin/env bash\nexit "${FAKE_CURL_EXIT:-0}"\n',
    )
    _write_executable(
        bin_dir / "systemctl",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    _write_executable(
        bin_dir / "ss",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    _write_executable(
        bin_dir / "lsusb",
        "#!/usr/bin/env bash\necho 'Stereolabs ZED 2'\n",
    )
    _write_executable(
        bin_dir / "journalctl",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    _write_executable(
        bin_dir / "tailscale",
        '#!/usr/bin/env bash\n'
        '[[ "${1:-}" == "ip" ]] && '
        'printf "%s" "${FAKE_TAILSCALE_IP-100.64.0.1}"\n'
        'exit 0\n',
    )
    mediamtx = tmp_path / "mediamtx"
    _write_executable(
        mediamtx,
        "#!/usr/bin/env bash\necho 'v1.19.2'\n",
    )

    env = os.environ.copy()
    env.update(
        {
            "SCOUT_WS": str(workspace),
            "MEDIAMTX_BIN": str(mediamtx),
            "CAMERA_TIMEOUT": "1",
            "RTSP_TIMEOUT": "1",
            "WEBRTC_TIMEOUT": "1",
        }
    )
    return env


def test_status_reports_each_healthy_stage(tmp_path):
    env = _fake_environment(tmp_path)
    env["FAKE_EGL_WARNING"] = "1"
    completed = subprocess.run(
        [str(HEALTH_SCRIPT), "--status"],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    assert completed.returncode == 0
    assert "PASS CAMERA_TOPIC" in completed.stdout
    assert "PASS RTSP_MEDIA" in completed.stdout
    assert "PASS WEBRTC_HTTP" in completed.stdout


def test_status_identifies_each_failed_stage(tmp_path):
    env = _fake_environment(tmp_path)
    env.update(
        {
            "FAKE_ROS_EXIT": "1",
            "FAKE_CODEC": "vp9",
            "FAKE_CURL_EXIT": "1",
        }
    )
    completed = subprocess.run(
        [str(HEALTH_SCRIPT), "--status"],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    assert completed.returncode == 1
    assert "FAIL CAMERA_TOPIC" in completed.stdout
    assert "FAIL RTSP_MEDIA" in completed.stdout
    assert "FAIL WEBRTC_HTTP" in completed.stdout


def test_diagnostic_check_propagates_unhealthy_exit(tmp_path):
    env = _fake_environment(tmp_path)
    env["FAKE_CURL_EXIT"] = "1"
    completed = subprocess.run(
        [str(CHECK_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    assert completed.returncode == 1
    assert "FAIL WEBRTC_HTTP" in completed.stdout
    assert "PASS CONFIG" in completed.stdout


def test_diagnostic_allows_lan_without_tailscale_ip(tmp_path):
    env = _fake_environment(tmp_path)
    env["FAKE_TAILSCALE_IP"] = ""
    completed = subprocess.run(
        [str(CHECK_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    assert completed.returncode == 0
    assert "SKIP ICE_NETWORK" in completed.stdout
    assert "LAN viewing remains supported" in completed.stdout


def test_stack_retries_transient_camera_preflight(tmp_path):
    count_file = tmp_path / "camera-count"
    health = _write_executable(
        tmp_path / "health.sh",
        "#!/usr/bin/env bash\n"
        "if [[ \"$1\" == --camera ]]; then\n"
        f"  count=$(cat '{count_file}' 2>/dev/null || echo 0)\n"
        "  count=$((count + 1))\n"
        f"  echo \"$count\" > '{count_file}'\n"
        "  (( count > 1 ))\n"
        "else\n"
        "  exit 0\n"
        "fi\n",
    )
    mediamtx = _write_executable(
        tmp_path / "mediamtx",
        "#!/usr/bin/env bash\nsleep 1\n",
    )
    env = os.environ.copy()
    env.update(
        {
            "HEALTH_SCRIPT": str(health),
            "MEDIAMTX_BIN": str(mediamtx),
            "SCOUTMINI_RUNTIME_DIR": str(tmp_path / "runtime"),
            "PREFLIGHT_ATTEMPTS": "2",
            "STARTUP_TIMEOUT": "1",
        }
    )

    completed = subprocess.run(
        [str(STACK_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    assert completed.returncode == 0
    assert count_file.read_text(encoding="utf-8").strip() == "2"
    assert "retrying" in completed.stderr


def test_stack_exits_cleanly_on_sigterm(tmp_path):
    health = _write_executable(
        tmp_path / "health.sh",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    mediamtx = _write_executable(
        tmp_path / "mediamtx",
        "#!/usr/bin/env bash\n"
        "trap 'exit 0' TERM INT\n"
        "while true; do sleep 1; done\n",
    )
    runtime_dir = tmp_path / "runtime"
    env = os.environ.copy()
    env.update(
        {
            "HEALTH_SCRIPT": str(health),
            "MEDIAMTX_BIN": str(mediamtx),
            "SCOUTMINI_RUNTIME_DIR": str(runtime_dir),
            "PREFLIGHT_ATTEMPTS": "1",
            "STARTUP_TIMEOUT": "1",
        }
    )
    process = subprocess.Popen(
        [str(STACK_SCRIPT)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    deadline = time.monotonic() + 5.0
    while not (runtime_dir / "webrtc.pid").exists():
        assert process.poll() is None
        assert time.monotonic() < deadline
        time.sleep(0.05)

    process.terminate()
    process.communicate(timeout=5.0)

    assert process.returncode == 0
    assert not (runtime_dir / "webrtc.pid").exists()
