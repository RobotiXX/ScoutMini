"""Process-level tests for Slack stream-control exit semantics."""

import os
from pathlib import Path
import stat
import subprocess


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROL_SCRIPT = PACKAGE_ROOT / "scripts" / "control_zed_stream.sh"


def _executable(path: Path, contents: str) -> Path:
    path.write_text(contents, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


def _environment(tmp_path: Path) -> dict[str, str]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    workspace = tmp_path / "ws"
    install = workspace / "install"
    install.mkdir(parents=True)
    (install / "setup.bash").write_text(
        f'export PATH="{bin_dir}:$PATH"\n',
        encoding="utf-8",
    )
    _executable(
        bin_dir / "ros2",
        "#!/usr/bin/env bash\n"
        "if [[ \"${1:-}\" == pkg ]]; then echo /fake/share; fi\n"
        "exit 0\n",
    )
    _executable(
        bin_dir / "systemctl",
        "#!/usr/bin/env bash\necho inactive\nexit 3\n",
    )
    _executable(bin_dir / "journalctl", "#!/usr/bin/env bash\nexit 0\n")
    health = _executable(
        tmp_path / "health.sh",
        "#!/usr/bin/env bash\necho 'FAIL WEBRTC_HTTP unavailable'\nexit 1\n",
    )
    check = _executable(
        tmp_path / "check.sh",
        "#!/usr/bin/env bash\necho 'FAIL RTSP_MEDIA unavailable'\nexit 1\n",
    )
    env = os.environ.copy()
    env.update(
        {
            "SCOUT_WS": str(workspace),
            "HEALTH_SCRIPT": str(health),
            "CHECK_SCRIPT": str(check),
        }
    )
    return env


def test_status_propagates_unhealthy_exit(tmp_path):
    completed = subprocess.run(
        [str(CONTROL_SCRIPT), "status"],
        check=False,
        capture_output=True,
        text=True,
        env=_environment(tmp_path),
    )

    assert completed.returncode == 1
    assert "FAIL WEBRTC_HTTP" in completed.stdout


def test_diagnostics_propagates_failed_stage(tmp_path):
    completed = subprocess.run(
        [str(CONTROL_SCRIPT), "diagnostics"],
        check=False,
        capture_output=True,
        text=True,
        env=_environment(tmp_path),
    )

    assert completed.returncode == 1
    assert "FAIL RTSP_MEDIA" in completed.stdout
