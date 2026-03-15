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

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
KILL_BEFORE_LAUNCH="${KILL_BEFORE_LAUNCH:-1}"

MAVROS_LAUNCH_FILE="${MAVROS_LAUNCH_FILE:-px4.launch}"
FCU_URL="${FCU_URL:-serial:///dev/ttyACM0:115200}"
MAVROS_RESPAWN="${MAVROS_RESPAWN:-true}"

RPLIDAR_SERIAL_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB0}"
RPLIDAR_BAUDRATE="${RPLIDAR_BAUDRATE:-115200}"
RPLIDAR_FRAME_ID="${RPLIDAR_FRAME_ID:-laser_frame}"
RPLIDAR_INVERTED="${RPLIDAR_INVERTED:-false}"
RPLIDAR_ANGLE_COMPENSATE="${RPLIDAR_ANGLE_COMPENSATE:-true}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"

ENABLE_ODOM_FLATTEN="${ENABLE_ODOM_FLATTEN:-1}"
ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/odom}"
ODOM_PARENT_FRAME="${ODOM_PARENT_FRAME:-odom}"
ODOM_CHILD_FRAME="${ODOM_CHILD_FRAME:-base_link}"

ENABLE_STATIC_TF="${ENABLE_STATIC_TF:-1}"
BASE_FRAME="${BASE_FRAME:-base_link}"
LIDAR_FRAME="${LIDAR_FRAME:-${RPLIDAR_FRAME_ID}}"
LIDAR_X="${LIDAR_X:-0.0}"
LIDAR_Y="${LIDAR_Y:-0.0}"
LIDAR_Z="${LIDAR_Z:-0.05}"
LIDAR_ROLL="${LIDAR_ROLL:-0.0}"
LIDAR_PITCH="${LIDAR_PITCH:-0.0}"
LIDAR_YAW="${LIDAR_YAW:-0.0}"

SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar.yaml}"

PLANNER_NODE="${PLANNER_NODE:-local_planner_mode_a}"
PLANNER_PARAMS_FILE="${PLANNER_PARAMS_FILE:-}"
PLANNER_SCAN_TOPIC="${PLANNER_SCAN_TOPIC:-${SCAN_TOPIC}}"
PLANNER_REMAP_CMD_VEL_TO="${PLANNER_REMAP_CMD_VEL_TO:-}"

MAVROS_LOG="${MAVROS_LOG:-/tmp/mavros_real_basic.log}"
RPLIDAR_LOG="${RPLIDAR_LOG:-/tmp/rplidar_real_basic.log}"
SLAM_LOG="${SLAM_LOG:-/tmp/slam_real_basic.log}"
ODOM_FLATTEN_LOG="${ODOM_FLATTEN_LOG:-/tmp/odom_flatten_real_basic.log}"
TF_LOG="${TF_LOG:-/tmp/lidar_static_tf_real_basic.log}"
PLANNER_LOG="${PLANNER_LOG:-/tmp/planner_real_basic.log}"

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
  kill_if_running "ros2 launch mavros ${MAVROS_LAUNCH_FILE}"
  kill_if_running "ros2 run rplidar_ros rplidar_composition"
  kill_if_running "ros2 launch slam_toolbox online_async_launch.py"
  kill_if_running "ros2 run odom_flatten px4_odom_flatten_node"
  kill_if_running "ros2 run obs_avoid local_planner_mode_a"
  kill_if_running "ros2 run tf2_ros static_transform_publisher.*${LIDAR_FRAME}"
}

start_mavros() {
  echo "[run] MAVROS -> ${MAVROS_LOG}"
  ros2 launch mavros "${MAVROS_LAUNCH_FILE}" \
    fcu_url:="${FCU_URL}" \
    respawn_mavros:="${MAVROS_RESPAWN}" \
    use_sim_time:=false >"${MAVROS_LOG}" 2>&1 &
  add_process "$!" "mavros"
}

start_rplidar() {
  echo "[run] rplidar_composition -> ${RPLIDAR_LOG}"
  ros2 run rplidar_ros rplidar_composition --ros-args \
    -p channel_type:=serial \
    -p serial_port:="${RPLIDAR_SERIAL_PORT}" \
    -p serial_baudrate:="${RPLIDAR_BAUDRATE}" \
    -p frame_id:="${RPLIDAR_FRAME_ID}" \
    -p inverted:="${RPLIDAR_INVERTED}" \
    -p angle_compensate:="${RPLIDAR_ANGLE_COMPENSATE}" \
    -p topic_name:="${SCAN_TOPIC#/}" >"${RPLIDAR_LOG}" 2>&1 &
  add_process "$!" "rplidar"
}

start_odom_flatten() {
  if ! is_true "${ENABLE_ODOM_FLATTEN}"; then
    return
  fi
  echo "[run] px4_odom_flatten_node -> ${ODOM_FLATTEN_LOG}"
  ros2 run odom_flatten px4_odom_flatten_node --ros-args \
    -p use_sim_time:=false \
    -p odom_topic:="${ODOM_TOPIC}" \
    -p parent_frame:="${ODOM_PARENT_FRAME}" \
    -p child_frame:="${ODOM_CHILD_FRAME}" >"${ODOM_FLATTEN_LOG}" 2>&1 &
  add_process "$!" "odom_flatten"
}

start_static_tf() {
  if ! is_true "${ENABLE_STATIC_TF}"; then
    return
  fi
  if [[ "${BASE_FRAME}" == "${LIDAR_FRAME}" ]]; then
    return
  fi

  echo "[run] static TF ${BASE_FRAME} -> ${LIDAR_FRAME} -> ${TF_LOG}"
  ros2 run tf2_ros static_transform_publisher \
    --x "${LIDAR_X}" --y "${LIDAR_Y}" --z "${LIDAR_Z}" \
    --roll "${LIDAR_ROLL}" --pitch "${LIDAR_PITCH}" --yaw "${LIDAR_YAW}" \
    --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_FRAME}" \
    --ros-args -p use_sim_time:=false >"${TF_LOG}" 2>&1 &
  add_process "$!" "lidar_static_tf"
}

start_slam() {
  echo "[run] slam_toolbox online_async_launch.py -> ${SLAM_LOG}"
  ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="${SLAM_PARAMS_FILE}" \
    use_sim_time:=false >"${SLAM_LOG}" 2>&1 &
  add_process "$!" "slam_toolbox"
}

start_planner() {
  local cmd=(ros2 run obs_avoid "${PLANNER_NODE}" --ros-args -p use_sim_time:=false)
  if [[ -n "${PLANNER_PARAMS_FILE}" ]]; then
    cmd+=(--params-file "${PLANNER_PARAMS_FILE}")
  fi
  if [[ -n "${PLANNER_REMAP_CMD_VEL_TO}" ]]; then
    cmd+=(-r "/mavros/setpoint_velocity/cmd_vel:=${PLANNER_REMAP_CMD_VEL_TO}")
  fi
  cmd+=(-r "/scan_horizontal:=${PLANNER_SCAN_TOPIC}")

  echo "[run] ${PLANNER_NODE} -> ${PLANNER_LOG}"
  "${cmd[@]}" >"${PLANNER_LOG}" 2>&1 &
  add_process "$!" "planner"
}

main() {
  require_cmd ros2

  case "${PLANNER_NODE}" in
    local_planner_mode_a) ;;
    *)
      echo "[error] for this basic stack, PLANNER_NODE must be local_planner_mode_a" >&2
      exit 1
      ;;
  esac

  if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
  fi
  if [[ ! -f "${SLAM_PARAMS_FILE}" ]]; then
    echo "[error] slam params file not found: ${SLAM_PARAMS_FILE}" >&2
    exit 1
  fi

  set +u
  source "${ROS_SETUP}"
  set -u

  if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
    echo "[error] mavros package not found in overlay." >&2
    exit 1
  fi
  if ! ros2 pkg prefix rplidar_ros >/dev/null 2>&1; then
    echo "[error] rplidar_ros package not found in overlay." >&2
    exit 1
  fi
  if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
    echo "[error] slam_toolbox package not found in overlay." >&2
    exit 1
  fi

  if [[ "${KILL_BEFORE_LAUNCH}" == "1" ]]; then
    echo "[prep] stopping stale real-basic processes (if any)"
    kill_existing_stack
    sleep 1
  fi

  trap cleanup EXIT INT TERM

  start_mavros
  echo "[wait] /mavros/state"
  wait_for_topic "/mavros/state" "${WAIT_TIMEOUT_SEC}"

  echo "[wait] ${ODOM_TOPIC}"
  wait_for_topic "${ODOM_TOPIC}" "${WAIT_TIMEOUT_SEC}"

  start_rplidar
  echo "[wait] ${SCAN_TOPIC}"
  wait_for_topic "${SCAN_TOPIC}" "${WAIT_TIMEOUT_SEC}"

  start_odom_flatten
  start_static_tf
  start_slam

  echo "[wait] /map"
  wait_for_topic "/map" "${WAIT_TIMEOUT_SEC}" || true

  start_planner

  echo "[ok] real basic 2D stack started"
  echo "[info] FCU_URL=${FCU_URL}"
  echo "[info] LiDAR port=${RPLIDAR_SERIAL_PORT} baud=${RPLIDAR_BAUDRATE} topic=${SCAN_TOPIC} frame=${RPLIDAR_FRAME_ID}"
  echo "[info] planner=${PLANNER_NODE} scan_remap=${PLANNER_SCAN_TOPIC}"
  echo "[info] use_sim_time=false"
  echo "[info] logs:"
  echo "  - ${MAVROS_LOG}"
  echo "  - ${RPLIDAR_LOG}"
  echo "  - ${SLAM_LOG}"
  echo "  - ${PLANNER_LOG}"
  if is_true "${ENABLE_ODOM_FLATTEN}"; then
    echo "  - ${ODOM_FLATTEN_LOG}"
  fi

  set +e
  wait -n "${PIDS[@]}"
  exit_code=$?
  set -e
  echo "[warn] one process exited (code=${exit_code}), stopping the others."
  exit "${exit_code}"
}

main "$@"
