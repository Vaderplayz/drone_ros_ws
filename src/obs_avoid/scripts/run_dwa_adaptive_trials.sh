#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

# setup.bash may reference COLCON_TRACE while unset; temporarily disable nounset.
set +u
source "${ROS_SETUP}"
set -u
export COLCON_TRACE="${COLCON_TRACE:-0}"

exec python3 "${SCRIPT_DIR}/dwa_adaptive_trials.py" "$@"

