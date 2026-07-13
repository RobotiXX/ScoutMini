"""Invoke the package's shared stream health script."""

import os

from ament_index_python.packages import get_package_share_directory


def main() -> None:
    script = os.path.join(
        get_package_share_directory('scoutmini_streaming'),
        'scripts',
        'stream_health.sh',
    )
    os.execv(str(script), [str(script), *os.sys.argv[1:]])
