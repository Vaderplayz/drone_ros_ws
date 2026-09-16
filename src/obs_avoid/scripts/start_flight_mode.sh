#!/usr/bin/env bash
# Starts the planner behind the six-direction spatial guard. This script never
# starts MAVROS, arms the vehicle, changes flight mode, or forwards to MAVROS.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

PLANNER_NODE="${PLANNER_NODE:-local_planner_mode_a}"
PLANNER_PARAMS_FILE="${PLANNER_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/local_planner_mode_a_real_safe.yaml}"
GUARD_PARAMS_FILE="${GUARD_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/spatial_command_guard_real.yaml}"
SPATIAL_STATUS_TOPIC="${SPATIAL_STATUS_TOPIC:-/mapping/spatial_awareness/status}"
SPATIAL_READY_TIMEOUT_SEC="${SPATIAL_READY_TIMEOUT_SEC:-10}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
ALLOW_LEGACY_PLANNER="${ALLOW_LEGACY_PLANNER:-0}"

case "${PLANNER_NODE}" in
  local_planner_mode_a) ;;
  local_planner_sector_mode|local_planner_hybrid_mode)
    if [[ "${ALLOW_LEGACY_PLANNER}" != "1" ]]; then
      echo "[error] ${PLANNER_NODE} has not been integrated with the spatial safety profile." >&2
      echo "        Use local_planner_mode_a, or explicitly set ALLOW_LEGACY_PLANNER=1 for bench work." >&2
      exit 1
    fi
    ;;
  *)
    echo "[error] invalid PLANNER_NODE='${PLANNER_NODE}'" >&2
    exit 1
    ;;
esac

for file in "${ROS_SETUP}" "${PLANNER_PARAMS_FILE}" "${GUARD_PARAMS_FILE}"; do
  if [[ ! -f "${file}" ]]; then
    echo "[error] required file not found: ${file}" >&2
    exit 1
  fi
done

set +u
source "${ROS_SETUP}"
set -u

if ! timeout "${SPATIAL_READY_TIMEOUT_SEC}" ros2 topic echo \
  "${SPATIAL_STATUS_TOPIC}" --once >/dev/null 2>&1; then
  echo "[error] no spatial-awareness diagnostic on ${SPATIAL_STATUS_TOPIC}." >&2
  echo "        Start the mapping/spatial-awareness pipeline before the planner." >&2
  exit 1
fi

GUARD_PID=""
cleanup() {
  set +e
  if [[ -n "${GUARD_PID}" ]] && kill -0 "${GUARD_PID}" 2>/dev/null; then
    kill "${GUARD_PID}" 2>/dev/null || true
    wait "${GUARD_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[run] spatial_command_guard_node: /planner_cmd_vel_raw -> /planner_cmd_vel"
ros2 run obs_avoid spatial_command_guard_node --ros-args \
  --params-file "${GUARD_PARAMS_FILE}" \
  -p use_sim_time:="${USE_SIM_TIME}" &
GUARD_PID="$!"
sleep 1
if ! kill -0 "${GUARD_PID}" 2>/dev/null; then
  echo "[error] spatial command guard exited during startup" >&2
  exit 1
fi

echo "[run] ${PLANNER_NODE} with conservative profile"
echo "[info] guarded output is /planner_cmd_vel; no direct MAVROS publisher is started"
set +e
ros2 run obs_avoid "${PLANNER_NODE}" --ros-args \
  --params-file "${PLANNER_PARAMS_FILE}" \
  -p use_sim_time:="${USE_SIM_TIME}" \
  -r /planner_cmd_vel:=/planner_cmd_vel_raw "$@"
status="$?"
set -e
exit "${status}"
