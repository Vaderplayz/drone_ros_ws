#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DEFAULT="$(cd "${PKG_DIR}/../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
FC_SERIAL_PATH="${FC_SERIAL_PATH:-/dev/serial/by-id/usb-MicoAir_MicoAir743AIO_0-if00}"
RPLIDAR_SERIAL_PORT="${RPLIDAR_SERIAL_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
RPLIDAR_BAUDRATE="${RPLIDAR_BAUDRATE:-115200}"
RPLIDAR_FRAME_ID="${RPLIDAR_FRAME_ID:-laser_frame}"
RPLIDAR_INVERTED="${RPLIDAR_INVERTED:-false}"
RPLIDAR_ANGLE_COMPENSATE="${RPLIDAR_ANGLE_COMPENSATE:-true}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
WEBCAM_PATH="${WEBCAM_PATH:-/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_HD_UVC_WebCam-video-index0}"

ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/odom}"
ODOM_PARENT_FRAME="${ODOM_PARENT_FRAME:-odom}"
ODOM_CHILD_FRAME="${ODOM_CHILD_FRAME:-base_link}"
BASE_FRAME="${BASE_FRAME:-base_link}"
LIDAR_FRAME="${LIDAR_FRAME:-${RPLIDAR_FRAME_ID}}"
LIDAR_X="${LIDAR_X:-0.0}"
LIDAR_Y="${LIDAR_Y:-0.0}"
LIDAR_Z="${LIDAR_Z:-0.05}"
LIDAR_ROLL="${LIDAR_ROLL:-0.0}"
LIDAR_PITCH="${LIDAR_PITCH:-0.0}"
LIDAR_YAW="${LIDAR_YAW:-0.0}"

SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar.yaml}"
PLANNER_PARAMS_FILE="${PLANNER_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/local_planner_mode_a_real_safe.yaml}"
PRECLAND_MODE="${PRECLAND_MODE:-}"
SETPOINT_HZ="${SETPOINT_HZ:-20.0}"
OFFBOARD_WARMUP_SEC="${OFFBOARD_WARMUP_SEC:-2.0}"
ODOM_TIMEOUT_SEC="${ODOM_TIMEOUT_SEC:-0.30}"
SCAN_TIMEOUT_SEC="${SCAN_TIMEOUT_SEC:-0.30}"
PLANNER_TIMEOUT_SEC="${PLANNER_TIMEOUT_SEC:-0.30}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROS_WS}/runtime_logs/real_slam_${RUN_STAMP}"
MASTER_LOG="${LOG_DIR}/master.log"
RPLIDAR_LOG="${LOG_DIR}/rplidar.log"
SLAM_LOG="${LOG_DIR}/slam_toolbox.log"
ODOM_FLATTEN_LOG="${LOG_DIR}/odom_flatten.log"
STATIC_TF_LOG="${LOG_DIR}/static_tf.log"
PLANNER_LOG="${LOG_DIR}/planner.log"
CONSOLE_LOG="${LOG_DIR}/command_console.log"
SYSTEM_SNAPSHOT="${LOG_DIR}/system_snapshot.txt"

PIDS=()
NAMES=()
MONITOR_PID=""
CONSOLE_PID=""
SHUTDOWN_REASON="normal console exit"

timestamp() {
  date '+%Y-%m-%dT%H:%M:%S%z'
}

log() {
  printf '[%s] %s\n' "$(timestamp)" "$*"
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    log "ERROR missing command: $1"
    exit 1
  fi
}

wait_for_topic() {
  local topic="$1"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      log "Topic available: ${topic}"
      return 0
    fi
    if (( $(date +%s) - start_ts >= WAIT_TIMEOUT_SEC )); then
      log "ERROR timed out waiting for topic: ${topic}"
      return 1
    fi
    sleep 1
  done
}

wait_for_service() {
  local service="$1"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | grep -Fxq "${service}"; then
      log "Service available: ${service}"
      return 0
    fi
    if (( $(date +%s) - start_ts >= WAIT_TIMEOUT_SEC )); then
      log "ERROR timed out waiting for service: ${service}"
      return 1
    fi
    sleep 1
  done
}

wait_for_mavros_connection() {
  local start_ts state
  start_ts="$(date +%s)"
  while true; do
    state="$(timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null || true)"
    if grep -Eq '^connected: true$' <<<"${state}"; then
      log "Existing MAVROS reports FCU connected"
      return 0
    fi
    if (( $(date +%s) - start_ts >= WAIT_TIMEOUT_SEC )); then
      log "ERROR MAVROS topic exists but FCU did not report connected"
      return 1
    fi
    sleep 1
  done
}

wait_for_map_message() {
  wait_for_topic /map
  if ! timeout "${WAIT_TIMEOUT_SEC}" ros2 topic echo /map --once >/dev/null 2>&1; then
    log "ERROR /map exists but no map message arrived"
    return 1
  fi
  log "Map message received"
}

record_device() {
  local label="$1"
  local path="$2"
  {
    printf '[%s] %s configured=%s\n' "$(timestamp)" "${label}" "${path}"
    if [[ -e "${path}" ]]; then
      printf 'resolved=%s\n' "$(readlink -f "${path}")"
      stat -Lc 'permissions=%A owner=%U group=%G device=%n' "${path}"
    else
      printf 'status=not-present\n'
    fi
  } >>"${SYSTEM_SNAPSHOT}"
}

add_process() {
  PIDS+=("$1")
  NAMES+=("$2")
  log "Started $2 pid=$1"
}

start_process() {
  local name="$1"
  local logfile="$2"
  shift 2
  "$@" >"${logfile}" 2>&1 &
  add_process "$!" "${name}"
}

monitor_processes() {
  declare -A reported=()
  while true; do
    local i
    for i in "${!PIDS[@]}"; do
      if [[ -z "${reported[${PIDS[$i]}]:-}" ]] && ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
        reported["${PIDS[$i]}"]=1
        log "Process exited: ${NAMES[$i]} pid=${PIDS[$i]}"
      fi
    done
    sleep 1
  done
}

cleanup() {
  local exit_code=$?
  set +e
  trap - EXIT INT TERM
  log "Shutdown reason: ${SHUTDOWN_REASON}; launcher_exit=${exit_code}"
  if [[ -n "${MONITOR_PID}" ]] && kill -0 "${MONITOR_PID}" 2>/dev/null; then
    kill "${MONITOR_PID}" 2>/dev/null
    wait "${MONITOR_PID}" 2>/dev/null
  fi
  if [[ -n "${CONSOLE_PID}" ]] && kill -0 "${CONSOLE_PID}" 2>/dev/null; then
    log "Stopping owned command console pid=${CONSOLE_PID}"
    kill "${CONSOLE_PID}" 2>/dev/null
    wait "${CONSOLE_PID}" 2>/dev/null
  fi
  local i pid status
  for ((i=${#PIDS[@]} - 1; i>=0; i--)); do
    pid="${PIDS[$i]}"
    if kill -0 "${pid}" 2>/dev/null; then
      log "Stopping owned process only: ${NAMES[$i]} pid=${pid}"
      kill "${pid}" 2>/dev/null
    fi
  done
  for i in "${!PIDS[@]}"; do
    status=0
    wait "${PIDS[$i]}" 2>/dev/null || status=$?
    log "Owned process final status: ${NAMES[$i]} pid=${PIDS[$i]} status=${status}"
  done
  log "MAVROS, FCU, webcam, AprilTag detector, and precision-landing pipeline were not stopped"
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 128
}

capture_runtime_snapshot() {
  {
    printf '\n[%s] ROS graph after startup\n' "$(timestamp)"
    printf '\n-- nodes --\n'
    ros2 node list
    printf '\n-- topics and types --\n'
    ros2 topic list -t
    printf '\n-- services and types --\n'
    ros2 service list -t
    printf '\n-- MAVROS state --\n'
    timeout 5 ros2 topic echo /mavros/state --once || true
    printf '\n-- local pose --\n'
    timeout 5 ros2 topic echo /mavros/local_position/pose --once || true
    printf '\n-- map availability --\n'
    ros2 topic info /map -v || true
    printf '\n-- scan rate --\n'
    timeout 8 ros2 topic hz /scan || true
    printf '\n-- odometry rate --\n'
    timeout 8 ros2 topic hz /mavros/local_position/odom || true
    printf '\n-- TF map to odom --\n'
    timeout 5 ros2 run tf2_ros tf2_echo map odom || true
    printf '\n-- TF odom to base_link --\n'
    timeout 5 ros2 run tf2_ros tf2_echo odom base_link || true
    printf '\n-- TF base_link to laser_frame --\n'
    timeout 5 ros2 run tf2_ros tf2_echo base_link laser_frame || true
  } >>"${SYSTEM_SNAPSHOT}" 2>&1
}

main() {
  mkdir -p "${LOG_DIR}"
  touch "${MASTER_LOG}" "${RPLIDAR_LOG}" "${SLAM_LOG}" "${ODOM_FLATTEN_LOG}" \
    "${STATIC_TF_LOG}" "${PLANNER_LOG}" "${CONSOLE_LOG}" "${SYSTEM_SNAPSHOT}"
  exec > >(tee -a "${MASTER_LOG}") 2>&1

  trap cleanup EXIT
  trap 'on_signal INT' INT
  trap 'on_signal TERM' TERM

  require_cmd ros2
  require_cmd timeout
  require_cmd readlink
  require_cmd stat

  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    log "ERROR ROS Jazzy setup not found"
    exit 1
  fi
  if [[ ! -f "${ROS_SETUP}" ]]; then
    log "ERROR workspace setup not found: ${ROS_SETUP}"
    exit 1
  fi
  if [[ ! -f "${SLAM_PARAMS_FILE}" || ! -f "${PLANNER_PARAMS_FILE}" ]]; then
    log "ERROR SLAM or planner parameter file missing"
    exit 1
  fi

  set +u
  # ROS overlay path is selected at runtime.
  source /opt/ros/jazzy/setup.bash
  source "${ROS_SETUP}"
  set -u

  {
    printf '[%s] real 2D LiDAR/SLAM run\n' "$(timestamp)"
    printf 'workspace=%s\n' "${ROS_WS}"
    printf 'git_branch=%s\n' "$(git -C "${ROS_WS}" branch --show-current)"
    printf 'git_commit=%s\n' "$(git -C "${ROS_WS}" rev-parse HEAD)"
    printf 'fc_path=%s\n' "${FC_SERIAL_PATH}"
    printf 'rplidar_path=%s baud=%s topic=%s frame=%s inverted=%s angle_compensate=%s\n' \
      "${RPLIDAR_SERIAL_PORT}" "${RPLIDAR_BAUDRATE}" "${SCAN_TOPIC}" "${RPLIDAR_FRAME_ID}" \
      "${RPLIDAR_INVERTED}" "${RPLIDAR_ANGLE_COMPENSATE}"
    printf 'webcam_path=%s\n' "${WEBCAM_PATH}"
    printf 'precland_mode=%s\n' "${PRECLAND_MODE:-NOT_CONFIGURED}"
  } >>"${SYSTEM_SNAPSHOT}"
  record_device "flight_controller" "${FC_SERIAL_PATH}"
  record_device "rplidar" "${RPLIDAR_SERIAL_PORT}"
  record_device "webcam" "${WEBCAM_PATH}"

  log "Reusing existing MAVROS and boot-time AprilTag pipeline; this launcher will not manage them"
  log "Selected FC path: ${FC_SERIAL_PATH} -> $(readlink -f "${FC_SERIAL_PATH}" 2>/dev/null || printf 'not-present')"
  log "Selected LiDAR path: ${RPLIDAR_SERIAL_PORT} -> $(readlink -f "${RPLIDAR_SERIAL_PORT}" 2>/dev/null || printf 'not-present')"
  log "Observed webcam path only (not opened): ${WEBCAM_PATH} -> $(readlink -f "${WEBCAM_PATH}" 2>/dev/null || printf 'not-present')"

  wait_for_topic /mavros/state
  wait_for_topic /mavros/local_position/pose
  wait_for_topic /mavros/local_position/odom
  wait_for_service /mavros/set_mode
  wait_for_service /mavros/cmd/arming
  wait_for_mavros_connection

  if [[ ! -e "${RPLIDAR_SERIAL_PORT}" ]]; then
    log "ERROR RPLIDAR stable device is absent: ${RPLIDAR_SERIAL_PORT}"
    exit 1
  fi
  if [[ ! -r "${RPLIDAR_SERIAL_PORT}" || ! -w "${RPLIDAR_SERIAL_PORT}" ]]; then
    log "ERROR RPLIDAR stable device is not readable and writable by $(id -un)"
    exit 1
  fi

  start_process rplidar "${RPLIDAR_LOG}" \
    ros2 run rplidar_ros rplidar_composition --ros-args \
      -p channel_type:=serial \
      -p serial_port:="${RPLIDAR_SERIAL_PORT}" \
      -p serial_baudrate:="${RPLIDAR_BAUDRATE}" \
      -p frame_id:="${RPLIDAR_FRAME_ID}" \
      -p inverted:="${RPLIDAR_INVERTED}" \
      -p angle_compensate:="${RPLIDAR_ANGLE_COMPENSATE}" \
      -p topic_name:="${SCAN_TOPIC#/}"
  wait_for_topic "${SCAN_TOPIC}"

  start_process odom_flatten "${ODOM_FLATTEN_LOG}" \
    ros2 run odom_flatten px4_odom_flatten_node --ros-args \
      -p use_sim_time:=false \
      -p odom_topic:="${ODOM_TOPIC}" \
      -p parent_frame:="${ODOM_PARENT_FRAME}" \
      -p child_frame:="${ODOM_CHILD_FRAME}"

  start_process static_tf "${STATIC_TF_LOG}" \
    ros2 run tf2_ros static_transform_publisher \
      --x "${LIDAR_X}" --y "${LIDAR_Y}" --z "${LIDAR_Z}" \
      --roll "${LIDAR_ROLL}" --pitch "${LIDAR_PITCH}" --yaw "${LIDAR_YAW}" \
      --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_FRAME}" \
      --ros-args -p use_sim_time:=false

  start_process slam_toolbox "${SLAM_LOG}" \
    ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:="${SLAM_PARAMS_FILE}" use_sim_time:=false
  wait_for_map_message

  start_process planner "${PLANNER_LOG}" \
    ros2 run obs_avoid local_planner_mode_a --ros-args \
      --params-file "${PLANNER_PARAMS_FILE}" \
      -p use_sim_time:=false \
      -r /scan_horizontal:="${SCAN_TOPIC}"

  log "Navigation stack ready. No arm, mode, or movement command has been sent."
  log "Runtime logs: ${LOG_DIR}"
  if [[ -z "${PRECLAND_MODE}" ]]; then
    log "WARNING precland command is disabled until PRECLAND_MODE is explicitly configured"
  fi

  console_cmd=(
    ros2 run obs_avoid real_slam_command_console --ros-args
    -p setpoint_hz:="${SETPOINT_HZ}"
    -p offboard_warmup_sec:="${OFFBOARD_WARMUP_SEC}"
    -p odom_timeout_sec:="${ODOM_TIMEOUT_SEC}"
    -p scan_timeout_sec:="${SCAN_TIMEOUT_SEC}"
    -p planner_timeout_sec:="${PLANNER_TIMEOUT_SEC}"
  )
  if [[ -n "${PRECLAND_MODE}" ]]; then
    console_cmd+=(-p precland_mode:="${PRECLAND_MODE}")
  fi
  if [[ ! -r /dev/tty ]]; then
    log "ERROR terminal console requires a controlling TTY"
    exit 1
  fi

  "${console_cmd[@]}" </dev/tty > >(tee -a "${CONSOLE_LOG}") 2>&1 &
  CONSOLE_PID="$!"
  log "Started command console pid=${CONSOLE_PID}"
  capture_runtime_snapshot &
  add_process "$!" system_snapshot
  monitor_processes &
  MONITOR_PID="$!"

  set +e
  wait "${CONSOLE_PID}"
  console_status=$?
  set -e
  log "Process exited: command_console pid=${CONSOLE_PID} status=${console_status}"
  SHUTDOWN_REASON="command console exited with status ${console_status}"
  return "${console_status}"
}

main "$@"
