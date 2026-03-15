#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

PLANNER_NODE="${PLANNER_NODE:-local_planner_mode_a}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

case "${PLANNER_NODE}" in
  local_planner_mode_a|local_planner_sector_mode|local_planner_hybrid_mode) ;;
  *)
    echo "[error] invalid PLANNER_NODE='${PLANNER_NODE}'" >&2
    echo "        allowed: local_planner_mode_a | local_planner_sector_mode | local_planner_hybrid_mode" >&2
    exit 1
    ;;
esac

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

echo "[run] ${PLANNER_NODE} (flight control + obstacle avoidance)"
exec ros2 run obs_avoid "${PLANNER_NODE}" --ros-args -p use_sim_time:="${USE_SIM_TIME}" "$@"
