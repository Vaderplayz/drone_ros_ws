#!/usr/bin/env bash

set -euo pipefail

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[error] missing command: $1" >&2
    exit 1
  fi
}

is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      return 0
    fi
    if (( "$(date +%s)" - start_ts >= timeout_sec )); then
      echo "[error] timed out waiting for topic: ${topic}" >&2
      return 1
    fi
    sleep 1
  done
}

kill_if_running() {
  local pattern="$1"
  pgrep -af "${pattern}" >/dev/null 2>&1 || return 0
  pkill -f "${pattern}" >/dev/null 2>&1 || true
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DEFAULT="$(cd "${PKG_DIR}/../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

PX4_DDS_LOCAL_POSITION_TOPIC="${PX4_DDS_LOCAL_POSITION_TOPIC:-/fmu/out/vehicle_local_position}"
PX4_DDS_GPS_TOPIC="${PX4_DDS_GPS_TOPIC:-/fmu/out/vehicle_gps_position}"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-45}"
KILL_BEFORE_LAUNCH="${KILL_BEFORE_LAUNCH:-1}"

ODOM_TOPIC="${ODOM_TOPIC:-/px4/odom}"
ODOM_PARENT_FRAME="${ODOM_PARENT_FRAME:-odom}"
ODOM_CHILD_FRAME="${ODOM_CHILD_FRAME:-base_link}"

ENABLE_STATIC_TF="${ENABLE_STATIC_TF:-1}"
BASE_FRAME="${BASE_FRAME:-base_link}"
LIDAR_VERT_FRAME="${LIDAR_VERT_FRAME:-lidar_vert_link}"
LIDAR_HORIZ_FRAME="${LIDAR_HORIZ_FRAME:-lidar_horiz_link}"
LIDAR_VERT_X="${LIDAR_VERT_X:-0.0}"
LIDAR_VERT_Y="${LIDAR_VERT_Y:-0.0}"
LIDAR_VERT_Z="${LIDAR_VERT_Z:-0.0}"
LIDAR_VERT_ROLL="${LIDAR_VERT_ROLL:-0.0}"
LIDAR_VERT_PITCH="${LIDAR_VERT_PITCH:--1.570796}"
LIDAR_VERT_YAW="${LIDAR_VERT_YAW:-0.0}"
LIDAR_HORIZ_X="${LIDAR_HORIZ_X:-0.0}"
LIDAR_HORIZ_Y="${LIDAR_HORIZ_Y:-0.0}"
LIDAR_HORIZ_Z="${LIDAR_HORIZ_Z:-0.0}"
LIDAR_HORIZ_ROLL="${LIDAR_HORIZ_ROLL:-0.0}"
LIDAR_HORIZ_PITCH="${LIDAR_HORIZ_PITCH:-0.0}"
LIDAR_HORIZ_YAW="${LIDAR_HORIZ_YAW:-0.0}"

PLANNER_NODE="${PLANNER_NODE:-local_planner_mode_a}"
PLANNER_PARAMS_FILE="${PLANNER_PARAMS_FILE:-}"
PLANNER_REMAP_CMD_VEL_TO="${PLANNER_REMAP_CMD_VEL_TO:-}"

START_MAPPING="${START_MAPPING:-0}"
MAPPING_PROFILE="${MAPPING_PROFILE:-v11_clean}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_vertical}"
TARGET_FRAME="${TARGET_FRAME:-map}"

PX4_DDS_BRIDGE_LOG="${PX4_DDS_BRIDGE_LOG:-/tmp/px4_dds_bridge_real_px4.log}"
PLANNER_LOG="${PLANNER_LOG:-/tmp/local_planner_real_px4.log}"
USER_CTRL_LOG="${USER_CTRL_LOG:-/tmp/user_ctrl_dds_real_px4.log}"
TF_VERT_LOG="${TF_VERT_LOG:-/tmp/static_tf_vert.log}"
TF_HORIZ_LOG="${TF_HORIZ_LOG:-/tmp/static_tf_horiz.log}"
MAPPING_LOG="${MAPPING_LOG:-/tmp/mapping_mode_real_px4.log}"

PIDS=()
NAMES=()

cleanup() {
  set +e
  local i
  for i in "${!PIDS[@]}"; do
    if kill -0 "${PIDS[$i]}" >/dev/null 2>&1; then
      echo "[stop] ${NAMES[$i]} (pid=${PIDS[$i]})"
      kill "${PIDS[$i]}" >/dev/null 2>&1 || true
    fi
  done
}

add_process() {
  local pid="$1"
  local name="$2"
  PIDS+=("${pid}")
  NAMES+=("${name}")
}

kill_existing_stack() {
  kill_if_running "ros2 run obs_avoid px4_dds_local_bridge"
  kill_if_running "ros2 run obs_avoid local_planner_mode_a"
  kill_if_running "ros2 run obs_avoid local_planner_sector_mode"
  kill_if_running "ros2 run obs_avoid local_planner_hybrid_mode"
  kill_if_running "ros2 run obs_avoid user_ctrl_dds"
  kill_if_running "ros2 run tf2_ros static_transform_publisher.*${LIDAR_VERT_FRAME}"
  kill_if_running "ros2 run tf2_ros static_transform_publisher.*${LIDAR_HORIZ_FRAME}"
  kill_if_running "scripts/start_mapping_mode.sh"
}

start_px4_dds_bridge() {
  echo "[run] px4_dds_local_bridge -> ${PX4_DDS_BRIDGE_LOG}"
  ros2 run obs_avoid px4_dds_local_bridge --ros-args \
    -p px4_local_position_topic:="${PX4_DDS_LOCAL_POSITION_TOPIC}" \
    -p px4_gps_topic:="${PX4_DDS_GPS_TOPIC}" \
    -p odom_topic:="${ODOM_TOPIC}" \
    -p odom_frame:="${ODOM_PARENT_FRAME}" \
    -p base_frame:="${ODOM_CHILD_FRAME}" >"${PX4_DDS_BRIDGE_LOG}" 2>&1 &
  add_process "$!" "px4_dds_bridge"
}

start_static_tf() {
  if ! is_true "${ENABLE_STATIC_TF}"; then
    return
  fi

  echo "[run] static TF ${BASE_FRAME} -> ${LIDAR_VERT_FRAME} -> ${TF_VERT_LOG}"
  ros2 run tf2_ros static_transform_publisher \
    --x "${LIDAR_VERT_X}" --y "${LIDAR_VERT_Y}" --z "${LIDAR_VERT_Z}" \
    --roll "${LIDAR_VERT_ROLL}" --pitch "${LIDAR_VERT_PITCH}" --yaw "${LIDAR_VERT_YAW}" \
    --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_VERT_FRAME}" \
    --ros-args -p use_sim_time:=false >"${TF_VERT_LOG}" 2>&1 &
  add_process "$!" "static_tf_vert"

  echo "[run] static TF ${BASE_FRAME} -> ${LIDAR_HORIZ_FRAME} -> ${TF_HORIZ_LOG}"
  ros2 run tf2_ros static_transform_publisher \
    --x "${LIDAR_HORIZ_X}" --y "${LIDAR_HORIZ_Y}" --z "${LIDAR_HORIZ_Z}" \
    --roll "${LIDAR_HORIZ_ROLL}" --pitch "${LIDAR_HORIZ_PITCH}" --yaw "${LIDAR_HORIZ_YAW}" \
    --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_HORIZ_FRAME}" \
    --ros-args -p use_sim_time:=false >"${TF_HORIZ_LOG}" 2>&1 &
  add_process "$!" "static_tf_horiz"
}

start_planner() {
  local cmd=(ros2 run obs_avoid "${PLANNER_NODE}" --ros-args -p use_sim_time:=false)
  if [[ -n "${PLANNER_PARAMS_FILE}" ]]; then
    cmd+=(--params-file "${PLANNER_PARAMS_FILE}")
  fi
  cmd+=(-p "odom_topic:=${ODOM_TOPIC}")
  if [[ -n "${PLANNER_REMAP_CMD_VEL_TO}" ]]; then
    cmd+=(-r "/planner_cmd_vel:=${PLANNER_REMAP_CMD_VEL_TO}")
  fi

  echo "[run] ${PLANNER_NODE} -> ${PLANNER_LOG}"
  "${cmd[@]}" >"${PLANNER_LOG}" 2>&1 &
  add_process "$!" "planner"
}

start_user_ctrl() {
  echo "[run] user_ctrl_dds (PX4 DDS OFFBOARD bridge mode) -> ${USER_CTRL_LOG}"
  ros2 run obs_avoid user_ctrl_dds --ros-args \
    -p use_sim_time:=false \
    -p ask_goal_on_start:=false \
    -p print_input_help_on_start:=false \
    -p enable_internal_goal_nav:=false \
    -p planner_cmd_timeout_sec:=1.0 >"${USER_CTRL_LOG}" 2>&1 &
  add_process "$!" "user_ctrl_dds"
}

start_mapping_mode() {
  if ! is_true "${START_MAPPING}"; then
    return
  fi
  echo "[run] start_mapping_mode.sh (use_sim_time=false) -> ${MAPPING_LOG}"
  USE_SIM_TIME=false \
  KILL_BEFORE_LAUNCH=0 \
  WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC}" \
  MAPPING_PROFILE="${MAPPING_PROFILE}" \
  SCAN_TOPIC="${SCAN_TOPIC}" \
  TARGET_FRAME="${TARGET_FRAME}" \
  "${SCRIPT_DIR}/start_mapping_mode.sh" >"${MAPPING_LOG}" 2>&1 &
  add_process "$!" "mapping_mode"
}

main() {
  require_cmd ros2

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

  if [[ "${KILL_BEFORE_LAUNCH}" == "1" ]]; then
    echo "[prep] stopping stale real-stack processes (if any)"
    kill_existing_stack
    sleep 1
  fi

  trap cleanup EXIT INT TERM

  start_px4_dds_bridge
  echo "[wait] ${PX4_DDS_LOCAL_POSITION_TOPIC}"
  wait_for_topic "${PX4_DDS_LOCAL_POSITION_TOPIC}" "${WAIT_TIMEOUT_SEC}"
  echo "[wait] ${ODOM_TOPIC}"
  wait_for_topic "${ODOM_TOPIC}" "${WAIT_TIMEOUT_SEC}"

  start_static_tf
  start_planner
  start_user_ctrl
  start_mapping_mode

  echo "[ok] real PX4 stack started"
  echo "[info] DDS local_position=${PX4_DDS_LOCAL_POSITION_TOPIC} odom=${ODOM_TOPIC}"
  echo "[info] planner=${PLANNER_NODE}"
  echo "[info] use_sim_time=false"
  echo "[info] logs:"
  echo "  - ${PX4_DDS_BRIDGE_LOG}"
  echo "  - ${PLANNER_LOG}"
  echo "  - ${USER_CTRL_LOG}"
  if is_true "${START_MAPPING}"; then
    echo "  - ${MAPPING_LOG}"
  fi
  echo "[check] ros2 topic hz /planner_cmd_vel"
  echo "[check] ros2 topic hz /fmu/in/trajectory_setpoint"
  echo "[check] ros2 topic echo --once /fmu/out/vehicle_local_position"

  set +e
  wait -n "${PIDS[@]}"
  exit_code=$?
  set -e
  echo "[warn] one process exited (code=${exit_code}), stopping the others."
  exit "${exit_code}"
}

main "$@"
