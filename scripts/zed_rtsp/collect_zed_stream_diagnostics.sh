#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/nvidia/repos/ScoutMini}"
OUT_DIR="${OUT_DIR:-$REPO_ROOT/scripts/zed_rtsp/diagnostics}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BUNDLE_DIR="$OUT_DIR/zed_stream_diag_$STAMP"
SCOUT_WS="${SCOUT_WS:-$REPO_ROOT/ros2_ws}"
ZED_RTSP_TOPIC="${ZED_RTSP_TOPIC:-/zed/zed_node/rgb/color/rect/image/compressed}"

mkdir -p "$BUNDLE_DIR"

run_capture() {
  local name="$1"
  shift
  {
    echo "\$ $*"
    "$@" 2>&1 || true
  } > "$BUNDLE_DIR/$name.txt"
}

run_shell_capture() {
  local name="$1"
  local command="$2"
  {
    echo "\$ $command"
    bash -lc "$command" 2>&1 || true
  } > "$BUNDLE_DIR/$name.txt"
}

run_capture host hostnamectl
run_capture host_ips hostname -I
run_capture date date --iso-8601=seconds
run_capture tailscale_status tailscale status
run_capture tailscale_ip tailscale ip -4
run_capture tailscale_netcheck tailscale netcheck
run_capture ssh_status systemctl status ssh --no-pager
run_capture tailscaled_status systemctl status tailscaled --no-pager
run_capture active_networks nmcli -t -f NAME,TYPE,DEVICE,STATE connection show --active
run_shell_capture ports "ss -ltnup | grep -E ':8554|:8889|:8189|:22' || true"
run_shell_capture processes "pgrep -af 'mediamtx|image2rtsp|zed_camera.launch.py|start_zed_.*stack|component_container_isolated' || true"
run_shell_capture git_status "cd '$REPO_ROOT' && git status --short docs scripts/zed_rtsp"
run_shell_capture recent_ros_logs "find /home/nvidia/.ros/log -maxdepth 2 -type f -mmin -120 | sort | tail -80"

if [[ -f /opt/ros/humble/setup.bash && -f "$SCOUT_WS/install/setup.bash" ]]; then
  run_shell_capture ros_topic_info "source /opt/ros/humble/setup.bash && source '$SCOUT_WS/install/setup.bash' && export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp} && timeout 8 ros2 topic info --no-daemon '$ZED_RTSP_TOPIC'"
  run_shell_capture ros_topic_list "source /opt/ros/humble/setup.bash && source '$SCOUT_WS/install/setup.bash' && export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp} && timeout 8 ros2 topic list --no-daemon | sort"
else
  echo "ROS setup files missing" > "$BUNDLE_DIR/ros_topic_info.txt"
fi

ARCHIVE="$OUT_DIR/zed_stream_diag_$STAMP.tar.gz"
tar -C "$OUT_DIR" -czf "$ARCHIVE" "$(basename "$BUNDLE_DIR")"

echo "Diagnostics written to:"
echo "  $BUNDLE_DIR"
echo "Archive:"
echo "  $ARCHIVE"
