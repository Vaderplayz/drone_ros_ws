#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u
export COLCON_TRACE="${COLCON_TRACE:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog}"
mkdir -p "${ROS_LOG_DIR}"

exec python3 "${SCRIPT_DIR}/trajectory_compare_node.py" "$@"
