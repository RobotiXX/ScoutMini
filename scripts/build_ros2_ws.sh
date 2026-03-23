#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2_ws"
SRC_DIR="${WS_DIR}/src"

ROS_DISTRO_DEFAULT="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_DEFAULT}/setup.bash"

WITH_ZED_DEBUG=0
CLEAN=0
EXTRA_ARGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [options] [extra colcon args...]

Build the ROS 2 workspace in this repository with sensible defaults:
  - symlink install
  - Release CMake build
  - BUILD_TESTING=OFF
  - console_direct event handler
  - auto-skip ZED packages if the ZED SDK is not installed
  - skip zed_debug by default

Options:
  --clean             Remove ${WS_DIR}/{build,install,log} before building
  --with-zed-debug    Include zed_debug in the build
  -h, --help          Show this help

Examples:
  $(basename "$0")
  $(basename "$0") --clean
  $(basename "$0") --packages-select scoutmini_nav2 scoutmini_bringup
  $(basename "$0") --with-zed-debug
EOF
}

while (($#)); do
  case "$1" in
    --clean)
      CLEAN=1
      shift
      ;;
    --with-zed-debug)
      WITH_ZED_DEBUG=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -d "${SRC_DIR}" ]]; then
  echo "Workspace source directory not found: ${SRC_DIR}" >&2
  exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"

if [[ ${CLEAN} -eq 1 ]]; then
  rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
fi

if [[ -z "${CUDA_TOOLKIT_ROOT_DIR:-}" && -d /usr/local/cuda ]]; then
  export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda
fi

if [[ -z "${ZED_DIR:-}" && -f /usr/local/zed/zed-config.cmake ]]; then
  export ZED_DIR=/usr/local/zed
fi

PACKAGES_SKIP=()

if [[ ! -f /usr/local/zed/zed-config.cmake ]]; then
  PACKAGES_SKIP+=(zed_components zed_wrapper zed_ros2 zed_debug)
  echo "ZED SDK not found under /usr/local/zed, skipping ZED packages."
elif [[ ${WITH_ZED_DEBUG} -eq 0 ]]; then
  PACKAGES_SKIP+=(zed_debug)
fi

COLCON_CMD=(
  colcon build
  --symlink-install
  --event-handlers console_direct+
  --parallel-workers "$(nproc)"
  --cmake-args
  -DCMAKE_BUILD_TYPE=Release
  -DBUILD_TESTING=OFF
)

if ((${#PACKAGES_SKIP[@]})); then
  COLCON_CMD+=(--packages-skip "${PACKAGES_SKIP[@]}")
fi

COLCON_CMD+=("${EXTRA_ARGS[@]}")

echo "Building workspace: ${WS_DIR}"
echo "ROS_DISTRO=${ROS_DISTRO_DEFAULT}"
echo "Command: ${COLCON_CMD[*]}"

cd "${WS_DIR}"
"${COLCON_CMD[@]}"

echo
echo "Build complete."
echo "Source this workspace with:"
echo "  source ${WS_DIR}/install/setup.bash"
