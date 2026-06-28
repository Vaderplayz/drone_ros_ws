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
PX4_DDS_LOCAL_POSITION_TOPIC="${PX4_DDS_LOCAL_POSITION_TOPIC:-/fmu/out/vehicle_local_position}"
PX4_DDS_GPS_TOPIC="${PX4_DDS_GPS_TOPIC:-/fmu/out/vehicle_gps_position}"

RPLIDAR_SERIAL_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB0}"
RPLIDAR_BAUDRATE="${RPLIDAR_BAUDRATE:-115200}"
RPLIDAR_FRAME_ID="${RPLIDAR_FRAME_ID:-laser_frame}"
RPLIDAR_INVERTED="${RPLIDAR_INVERTED:-false}"
RPLIDAR_ANGLE_COMPENSATE="${RPLIDAR_ANGLE_COMPENSATE:-true}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"

ENABLE_PX4_DDS_BRIDGE="${ENABLE_PX4_DDS_BRIDGE:-1}"
ODOM_TOPIC="${ODOM_TOPIC:-/px4/odom}"
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
MAX_ALTITUDE_M="${MAX_ALTITUDE_M:-5.0}"
MIN_ALTITUDE_M="${MIN_ALTITUDE_M:-0.0}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-3.0}"
MAX_YAW_RATE_RAD_S="${MAX_YAW_RATE_RAD_S:-0.45}"

PX4_DDS_BRIDGE_LOG="${PX4_DDS_BRIDGE_LOG:-/tmp/px4_dds_bridge_real_basic.log}"
RPLIDAR_LOG="${RPLIDAR_LOG:-/tmp/rplidar_real_basic.log}"
SLAM_LOG="${SLAM_LOG:-/tmp/slam_real_basic.log}"
TF_LOG="${TF_LOG:-/tmp/lidar_static_tf_real_basic.log}"
PLANNER_LOG="${PLANNER_LOG:-/tmp/planner_real_basic.log}"
USER_CTRL_LOG="${USER_CTRL_LOG:-/tmp/user_ctrl_dds_real_basic.log}"

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
  kill_if_running "ros2 run rplidar_ros rplidar_composition"
  kill_if_running "ros2 launch slam_toolbox online_async_launch.py"
  kill_if_running "ros2 run obs_avoid local_planner_mode_a"
  kill_if_running "ros2 run obs_avoid user_ctrl_dds"
  kill_if_running "ros2 run tf2_ros static_transform_publisher.*${LIDAR_FRAME}"
}

start_px4_dds_bridge() {
  if ! is_true "${ENABLE_PX4_DDS_BRIDGE}"; then
    return
  fi
  echo "[run] px4_dds_local_bridge -> ${PX4_DDS_BRIDGE_LOG}"
  ros2 run obs_avoid px4_dds_local_bridge --ros-args \
    -p px4_local_position_topic:="${PX4_DDS_LOCAL_POSITION_TOPIC}" \
    -p px4_gps_topic:="${PX4_DDS_GPS_TOPIC}" \
    -p odom_topic:="${ODOM_TOPIC}" \
    -p odom_frame:="${ODOM_PARENT_FRAME}" \
    -p base_frame:="${ODOM_CHILD_FRAME}" >"${PX4_DDS_BRIDGE_LOG}" 2>&1 &
  add_process "$!" "px4_dds_bridge"
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
  cmd+=(
    -p "v_max:=0.9"
    -p "vy_max:=0.6"
    -p "w_max:=${MAX_YAW_RATE_RAD_S}"
    -p "vz_max:=0.6"
    -p "ax_max:=0.8"
    -p "ay_max:=0.8"
    -p "aw_max:=1.0"
    -p "enable_sharp_turn_assist:=false"
    -p "bypass_strafe_speed:=0.25"
    -p "bypass_forward_speed:=0.18"
    -p "bypass_yaw_rate:=0.25"
    -p "max_linear_speed_total:=${MAX_LINEAR_SPEED_MPS}"
    -p "max_yaw_rate_abs:=${MAX_YAW_RATE_RAD_S}"
    -p "max_altitude_m:=${MAX_ALTITUDE_M}"
    -p "min_altitude_m:=${MIN_ALTITUDE_M}"
    -p "odom_topic:=${ODOM_TOPIC}"
  )
  if [[ -n "${PLANNER_PARAMS_FILE}" ]]; then
    cmd+=(--params-file "${PLANNER_PARAMS_FILE}")
  fi
  if [[ -n "${PLANNER_REMAP_CMD_VEL_TO}" ]]; then
    cmd+=(-r "/planner_cmd_vel:=${PLANNER_REMAP_CMD_VEL_TO}")
  fi
  cmd+=(-r "/scan_horizontal:=${PLANNER_SCAN_TOPIC}")

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
    -p planner_cmd_timeout_sec:=1.0 \
    -p max_altitude_m:="${MAX_ALTITUDE_M}" \
    -p min_altitude_m:="${MIN_ALTITUDE_M}" \
    -p max_linear_speed_mps:="${MAX_LINEAR_SPEED_MPS}" \
    -p max_yaw_rate_rad_s:="${MAX_YAW_RATE_RAD_S}" \
    -p max_accel_xy_mps2:=0.8 \
    -p max_accel_z_mps2:=0.6 \
    -p max_yaw_accel_rad_s2:=1.0 >"${USER_CTRL_LOG}" 2>&1 &
  add_process "$!" "user_ctrl_dds"
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

  start_px4_dds_bridge
  echo "[wait] ${PX4_DDS_LOCAL_POSITION_TOPIC}"
  wait_for_topic "${PX4_DDS_LOCAL_POSITION_TOPIC}" "${WAIT_TIMEOUT_SEC}"

  echo "[wait] ${ODOM_TOPIC}"
  wait_for_topic "${ODOM_TOPIC}" "${WAIT_TIMEOUT_SEC}"

  start_rplidar
  echo "[wait] ${SCAN_TOPIC}"
  wait_for_topic "${SCAN_TOPIC}" "${WAIT_TIMEOUT_SEC}"

  start_static_tf
  start_slam

  echo "[wait] /map"
  wait_for_topic "/map" "${WAIT_TIMEOUT_SEC}" || true

  start_planner
  start_user_ctrl

  echo "[ok] real basic 2D stack started"
  echo "[info] LiDAR port=${RPLIDAR_SERIAL_PORT} baud=${RPLIDAR_BAUDRATE} topic=${SCAN_TOPIC} frame=${RPLIDAR_FRAME_ID}"
  echo "[info] planner=${PLANNER_NODE} scan_remap=${PLANNER_SCAN_TOPIC}"
  echo "[info] DDS local_position=${PX4_DDS_LOCAL_POSITION_TOPIC} odom=${ODOM_TOPIC}"
  echo "[info] safety: max_altitude=${MAX_ALTITUDE_M}m max_linear=${MAX_LINEAR_SPEED_MPS}m/s max_yaw_rate=${MAX_YAW_RATE_RAD_S}rad/s"
  echo "[info] use_sim_time=false"
  echo "[info] logs:"
  echo "  - ${PX4_DDS_BRIDGE_LOG}"
  echo "  - ${RPLIDAR_LOG}"
  echo "  - ${SLAM_LOG}"
  echo "  - ${PLANNER_LOG}"
  echo "  - ${USER_CTRL_LOG}"

  set +e
  wait -n "${PIDS[@]}"
  exit_code=$?
  set -e
  echo "[warn] one process exited (code=${exit_code}), stopping the others."
  exit "${exit_code}"
}

main "$@"
