#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091

set -euo pipefail

ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
LIDAR_MODE="${LIDAR_MODE:-mapping_only}"
SLAM_PROFILE="${SLAM_PROFILE:-normal}"
USE_DESKEWED_SCAN="${USE_DESKEWED_SCAN:-0}"
START_LIDAR_PX4_BRIDGE="${START_LIDAR_PX4_BRIDGE:-0}"
START_RF2O_OBSERVER="${START_RF2O_OBSERVER:-0}"
START_SLAM_IN_RF2O_VALIDATION="${START_SLAM_IN_RF2O_VALIDATION:-1}"
RECORD_LIDAR_DIAGNOSTIC_BAG="${RECORD_LIDAR_DIAGNOSTIC_BAG:-${RECORD_BAG:-0}}"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
RPLIDAR_START_RETRIES=2
REUSE_EXISTING_RPLIDAR="${REUSE_EXISTING_RPLIDAR:-1}"
RPLIDAR_SCAN_WAIT_SEC="${RPLIDAR_SCAN_WAIT_SEC:-60}"
RPLIDAR_RETRY_DELAY_SEC="${RPLIDAR_RETRY_DELAY_SEC:-3}"
PROCESS_STOP_TIMEOUT_SEC="${PROCESS_STOP_TIMEOUT_SEC:-5}"
RPLIDAR_SERIAL_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB0}"
RPLIDAR_BAUDRATE="${RPLIDAR_BAUDRATE:-115200}"
RPLIDAR_SCAN_MODE="${RPLIDAR_SCAN_MODE:-Standard}"
RPLIDAR_FRAME_ID="${RPLIDAR_FRAME_ID:-laser_frame}"
RPLIDAR_INVERTED="${RPLIDAR_INVERTED:-false}"
RPLIDAR_ANGLE_COMPENSATE="${RPLIDAR_ANGLE_COMPENSATE:-true}"

SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
SCAN_AUDIT_DIAGNOSTICS_TOPIC="${SCAN_AUDIT_DIAGNOSTICS_TOPIC:-/scan_stream_audit/diagnostics}"
RF2O_SCAN_TOPIC="${RF2O_SCAN_TOPIC:-/scan_rf2o}"
RF2O_SCAN_DIAGNOSTICS_TOPIC="${RF2O_SCAN_DIAGNOSTICS_TOPIC:-/scan_rf2o/diagnostics}"
DESKEWED_SCAN_TOPIC="${DESKEWED_SCAN_TOPIC:-/scan_deskewed}"
DESKEW_DIAGNOSTICS_TOPIC="${DESKEW_DIAGNOSTICS_TOPIC:-/scan_deskewed/diagnostics}"
SCAN_TF_DIAGNOSTICS_TOPIC="${SCAN_TF_DIAGNOSTICS_TOPIC:-/scan_tf_timing_audit/diagnostics}"
RF2O_SCAN_BINS="${RF2O_SCAN_BINS:-720}"
SCAN_AUDIT_WAIT_SEC="${SCAN_AUDIT_WAIT_SEC:-30}"
CANONICAL_SCAN_WAIT_SEC="${CANONICAL_SCAN_WAIT_SEC:-45}"
SCAN_TF_WAIT_SEC="${SCAN_TF_WAIT_SEC:-30}"
DESKEW_SCAN_WAIT_SEC="${DESKEW_SCAN_WAIT_SEC:-30}"
RF2O_ODOM_WAIT_SEC="${RF2O_ODOM_WAIT_SEC:-30}"
RF2O_RAW_ODOM_TOPIC="${RF2O_RAW_ODOM_TOPIC:-/lidar/odom_raw}"
LIDAR_ODOM_TOPIC="${LIDAR_ODOM_TOPIC:-/lidar/odom}"
LIDAR_ODOM_DIAGNOSTICS_TOPIC="${LIDAR_ODOM_DIAGNOSTICS_TOPIC:-/lidar_odom/diagnostics}"
PX4_ODOMETRY_OUT_TOPIC="${PX4_ODOMETRY_OUT_TOPIC:-/mavros/odometry/out}"
LIDAR_PX4_DIAGNOSTICS_TOPIC="${LIDAR_PX4_DIAGNOSTICS_TOPIC:-/lidar_odom_px4_bridge/diagnostics}"

ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/odom}"
ODOM_PARENT_FRAME="${ODOM_PARENT_FRAME:-odom}"
ODOM_CHILD_FRAME="${ODOM_CHILD_FRAME:-base_footprint}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
LIDAR_FRAME="${LIDAR_FRAME:-${RPLIDAR_FRAME_ID}}"
LIDAR_X="${LIDAR_X:-0.0}"
LIDAR_Y="${LIDAR_Y:-0.0}"
LIDAR_Z="${LIDAR_Z:-0.1}"
LIDAR_ROLL="${LIDAR_ROLL:-0.0}"
LIDAR_PITCH="${LIDAR_PITCH:-0.0}"
LIDAR_YAW="${LIDAR_YAW:-0.0}"

SLAM_NORMAL_PARAMS_FILE="${SLAM_NORMAL_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar.yaml}"
SLAM_TIMING_PARAMS_FILE="${SLAM_TIMING_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar_timing_debug.yaml}"
PLANNER_PARAMS_FILE="${PLANNER_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/local_planner_mode_a_real_safe.yaml}"
LIDAR_MONITOR_PARAMS_FILE="${LIDAR_MONITOR_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/lidar_odom_px4_bridge.yaml}"
RF2O_PARAMS_FILE="${RF2O_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/rf2o_real_a1m8.yaml}"
PRECLAND_MODE="${PRECLAND_MODE:-}"
SETPOINT_HZ="${SETPOINT_HZ:-20.0}"
OFFBOARD_WARMUP_SEC="${OFFBOARD_WARMUP_SEC:-2.0}"
ODOM_TIMEOUT_SEC="${ODOM_TIMEOUT_SEC:-0.30}"
SCAN_TIMEOUT_SEC="${SCAN_TIMEOUT_SEC:-0.30}"
PLANNER_TIMEOUT_SEC="${PLANNER_TIMEOUT_SEC:-0.30}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_OWNER_ID="real_slam_${RUN_STAMP}_$$_${RANDOM}"
LOG_DIR="${ROS_WS}/runtime_logs/real_slam_${RUN_STAMP}"
MASTER_LOG="${LOG_DIR}/master.log"
SYSTEM_SNAPSHOT="${LOG_DIR}/system_snapshot.txt"
RPLIDAR_LOG="${LOG_DIR}/rplidar.log"
SCAN_AUDIT_LOG="${LOG_DIR}/scan_stream_audit.log"
RAW_SCAN_AUDIT_CSV="${LOG_DIR}/raw_scan_audit.csv"
CANONICALIZER_LOG="${LOG_DIR}/canonicalizer.log"
CANONICAL_SCAN_HEALTH_CSV="${LOG_DIR}/canonical_scan_health.csv"
RF2O_LOG="${LOG_DIR}/rf2o.log"
LIDAR_ODOM_MONITOR_LOG="${LOG_DIR}/lidar_odom_monitor.log"
LIDAR_ODOM_HEALTH_CSV="${LOG_DIR}/lidar_odom_health.csv"
SLAM_LOG="${LOG_DIR}/slam_toolbox.log"
ODOM_FLATTEN_LOG="${LOG_DIR}/odom_flatten.log"
STATIC_TF_LOG="${LOG_DIR}/static_tf.log"
PLANNER_LOG="${LOG_DIR}/planner.log"
CONSOLE_LOG="${LOG_DIR}/command_console.log"
PX4_BRIDGE_LOG="${LOG_DIR}/lidar_px4_bridge.log"
PX4_BRIDGE_HEALTH_CSV="${LOG_DIR}/lidar_px4_bridge_health.csv"
ROSBAG_LOG="${LOG_DIR}/rosbag.log"
PROCESS_HEALTH_CSV="${LOG_DIR}/process_health.csv"
TF_HEALTH_CSV="${LOG_DIR}/tf_health.csv"
SCAN_TF_TIMING_LOG="${LOG_DIR}/scan_tf_timing_audit.log"
SCAN_TF_TIMING_CSV="${LOG_DIR}/scan_tf_timing_audit.csv"
SCAN_MOTION_CSV="${LOG_DIR}/scan_motion_diagnostics.csv"
DESKEWER_LOG="${LOG_DIR}/laser_scan_deskewer.log"
DESKEW_HEALTH_CSV="${LOG_DIR}/deskew_health.csv"

if [[ "${USE_DESKEWED_SCAN}" == "1" ]]; then
  SELECTED_SCAN_TOPIC="${DESKEWED_SCAN_TOPIC}"
else
  SELECTED_SCAN_TOPIC="${RF2O_SCAN_TOPIC}"
fi
if [[ "${SLAM_PROFILE}" == "timing_debug" ]]; then
  SLAM_PARAMS_FILE="${SLAM_TIMING_PARAMS_FILE}"
else
  SLAM_PARAMS_FILE="${SLAM_NORMAL_PARAMS_FILE}"
fi

COMPONENTS=(
  MAVROS RPLIDAR RAW_SCAN ODOM_FLATTEN STATIC_TF SCAN_AUDIT CANONICALIZER
  CANONICAL_SCAN SCAN_TF_AUDIT DESKEWER DESKEWED_SCAN RF2O RF2O_MONITOR
  SLAM MAP PLANNER PX4_BRIDGE
)
PIDS=()
NAMES=()
TOKENS=()
LAST_STARTED_PID=""
EXISTING_RPLIDAR_PID=""
CONSOLE_PID=""
CONSOLE_TOKEN=""
PROCESS_MONITOR_PID=""
HEALTH_MONITOR_PID=""
TF_MONITOR_PID=""
SNAPSHOT_PID=""
RPLIDAR_PROBE_TOKEN=""
SHUTDOWN_REASON="normal exit"
LAUNCH_START_EPOCH="$(date +%s)"
declare -A COMPONENT_STATE=()
declare -A COMPONENT_START_EPOCH=()

timestamp() {
  date '+%Y-%m-%dT%H:%M:%S%z'
}

log() {
  printf '[%s] %s\n' "$(timestamp)" "$*"
}

set_component_state() {
  local component="$1"
  local state="$2"
  local detail="${3:-}"
  local now_epoch old_state start_epoch elapsed
  now_epoch="$(date +%s)"
  old_state="${COMPONENT_STATE[${component}]:-NOT_STARTED}"
  if [[ "${state}" == "STARTING" ]]; then
    COMPONENT_START_EPOCH["${component}"]="${now_epoch}"
  fi
  start_epoch="${COMPONENT_START_EPOCH[${component}]:-${LAUNCH_START_EPOCH}}"
  elapsed=$((now_epoch - start_epoch))
  COMPONENT_STATE["${component}"]="${state}"
  log "${component}_STATE ${old_state}->${state} elapsed=${elapsed}s${detail:+ detail=${detail}}"
}

initialize_component_states() {
  local component
  for component in "${COMPONENTS[@]}"; do
    COMPONENT_STATE["${component}"]="NOT_STARTED"
    COMPONENT_START_EPOCH["${component}"]="${LAUNCH_START_EPOCH}"
    log "${component}_STATE NOT_STARTED elapsed=0s"
  done
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    log "ERROR missing command: $1"
    exit 1
  fi
}

publisher_count() {
  local topic="$1"
  local info count
  info="$(ros2 topic info "${topic}" -v 2>/dev/null || true)"
  count="$(awk '/Publisher count:/{print $3; exit}' <<<"${info}")"
  printf '%s\n' "${count:-0}"
}

require_publisher_count() {
  local topic="$1"
  local expected="$2"
  local actual
  actual="$(publisher_count "${topic}")"
  if [[ "${actual}" != "${expected}" ]]; then
    log "ERROR topic ${topic} publisher_count=${actual}, expected=${expected}"
    ros2 topic info "${topic}" -v 2>/dev/null || true
    return 1
  fi
  log "Publisher count verified: ${topic}=${actual}"
}

discover_existing_rplidar() {
  local -a processes=()
  mapfile -t processes < <(pgrep -af '[r]plidar_composition|[r]plidar_node' 2>/dev/null || true)
  if (( ${#processes[@]} > 1 )); then
    log "ERROR multiple existing RPLIDAR processes detected:"
    printf '%s\n' "${processes[@]}"
    return 1
  fi
  if (( ${#processes[@]} == 0 )); then
    return 0
  fi
  if [[ "${REUSE_EXISTING_RPLIDAR}" != "1" ]]; then
    log "ERROR existing RPLIDAR process detected and reuse is disabled:"
    printf '%s\n' "${processes[0]}"
    return 1
  fi
  if [[ "${processes[0]}" != *"serial_port:=${RPLIDAR_SERIAL_PORT}"* ]]; then
    log "ERROR existing RPLIDAR does not use configured port ${RPLIDAR_SERIAL_PORT}:"
    printf '%s\n' "${processes[0]}"
    return 1
  fi
  EXISTING_RPLIDAR_PID="${processes[0]%% *}"
  printf '%s\n' "${EXISTING_RPLIDAR_PID}" >"${LOG_DIR}/external_rplidar.pid"
  log "Reusing one existing compatible RPLIDAR process; pid=${EXISTING_RPLIDAR_PID}"
}

require_no_duplicate_processes() {
  local pattern output
  local patterns=(
    '[l]aser_scan_stream_audit' '[l]aser_scan_canonicalizer'
    '[s]can_tf_timing_audit' '[l]aser_scan_deskewer'
    '[r]f2o_laser_odometry_node' '[l]idar_odom_monitor' '[a]sync_slam_toolbox_node'
    '[p]x4_odom_flatten_node' '[l]idar_odom_px4_bridge'
  )
  for pattern in "${patterns[@]}"; do
    output="$(pgrep -af "${pattern}" 2>/dev/null || true)"
    if [[ -n "${output}" ]]; then
      log "ERROR existing pipeline process detected for pattern ${pattern}:"
      printf '%s\n' "${output}"
      return 1
    fi
  done
  discover_existing_rplidar
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
  local pid="$1"
  local name="$2"
  local token="${3:-}"
  PIDS+=("${pid}")
  NAMES+=("${name}")
  TOKENS+=("${token}")
  printf '%s\n' "${pid}" >"${LOG_DIR}/${name}.pid"
  log "Started ${name} pid=${pid}"
}

start_process() {
  local name="$1"
  local logfile="$2"
  local token="${LAUNCH_OWNER_ID}:${name}:${#PIDS[@]}"
  shift 2
  setsid env REAL_SLAM_OWNER_TOKEN="${token}" "$@" >"${logfile}" 2>&1 &
  LAST_STARTED_PID="$!"
  add_process "${LAST_STARTED_PID}" "${name}" "${token}"
}

owned_pids_for_token() {
  local token="$1"
  local proc pid environment
  for proc in /proc/[0-9]*; do
    pid="${proc##*/}"
    if [[ -r "${proc}/environ" ]] &&
      environment="$(dd if="${proc}/environ" status=none 2>/dev/null | tr '\0' '\n')" &&
      grep -Fxq "REAL_SLAM_OWNER_TOKEN=${token}" <<<"${environment}"; then
      printf '%s\n' "${pid}"
    fi
  done
}

stop_owned_token() {
  local token="$1"
  local label="$2"
  local deadline
  local -a owned=()
  [[ -n "${token}" ]] || return 0
  mapfile -t owned < <(owned_pids_for_token "${token}")
  if (( ${#owned[@]} == 0 )); then
    return 0
  fi
  log "Stopping launcher-owned process token only: ${label}; process_count=${#owned[@]}"
  kill -TERM "${owned[@]}" 2>/dev/null || true
  deadline=$(( $(date +%s) + PROCESS_STOP_TIMEOUT_SEC ))
  while true; do
    mapfile -t owned < <(owned_pids_for_token "${token}")
    if (( ${#owned[@]} == 0 )); then
      return 0
    fi
    if (( $(date +%s) >= deadline )); then
      log "Launcher-owned ${label} did not stop after ${PROCESS_STOP_TIMEOUT_SEC}s; sending KILL"
      kill -KILL "${owned[@]}" 2>/dev/null || true
      return 0
    fi
    sleep 0.1
  done
}

stop_process_group() {
  local pid="$1"
  local index
  for index in "${!PIDS[@]}"; do
    if [[ "${PIDS[$index]}" == "${pid}" ]]; then
      stop_owned_token "${TOKENS[$index]}" "${NAMES[$index]}"
      return 0
    fi
  done
  if [[ -n "${CONSOLE_PID}" && "${CONSOLE_PID}" == "${pid}" ]]; then
    stop_owned_token "${CONSOLE_TOKEN}" "command_console"
    return 0
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    log "WARNING refusing to stop unowned pid=${pid}; no launcher ownership token"
  fi
}

wait_for_topic_message_alive() {
  local topic="$1"
  local pid="$2"
  local timeout_sec="$3"
  local logfile="$4"
  local start_ts elapsed last_progress=-1
  start_ts="$(date +%s)"
  while true; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      log "ERROR process pid=${pid} exited before publishing ${topic}"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 10
    fi
    if timeout 2 ros2 topic echo "${topic}" --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
      log "Message received: ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
      log "ERROR no ${topic} message after ${timeout_sec}s; process pid=${pid} remains alive"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic}; elapsed=${elapsed}s deadline=${timeout_sec}s pid=${pid}"
    fi
    sleep 1
  done
}

rplidar_failure_in_log() {
  grep -Eqi \
    'fatal|cannot open|failed to open|permission denied|device busy|SDK error|bind failed|serial.*error|cannot start scan|80008000|operation timeout' \
    "$1" 2>/dev/null
}

rplidar_start_scan_timeout_in_log() {
  grep -Eqi 'cannot start scan|80008000|operation timeout' "$1" 2>/dev/null
}

wait_for_rplidar_scan() {
  local pid="$1"
  local timeout_sec="$2"
  local logfile="$3"
  local start_ts elapsed last_progress=-1 probe_pid probe_status
  RPLIDAR_PROBE_TOKEN="${LAUNCH_OWNER_ID}:rplidar_scan_probe"
  env REAL_SLAM_OWNER_TOKEN="${RPLIDAR_PROBE_TOKEN}" \
    python3 - "${SCAN_TOPIC}" <<'PY' >/dev/null 2>&1 &
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


rclpy.init()
node = Node(f"rplidar_startup_scan_probe_{os.getpid()}")
received = False


def callback(_msg):
    global received
    received = True
    rclpy.shutdown()


subscription = node.create_subscription(
    LaserScan, sys.argv[1], callback, qos_profile_sensor_data)
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
sys.exit(0 if received else 1)
PY
  probe_pid="$!"
  start_ts="$(date +%s)"
  while true; do
    if rplidar_failure_in_log "${logfile}"; then
      kill "${probe_pid}" 2>/dev/null || true
      wait "${probe_pid}" 2>/dev/null || true
      if rplidar_start_scan_timeout_in_log "${logfile}"; then
        log "ERROR RPLIDAR reported a start-scan SDK timeout (80008000)"
      else
        log "ERROR RPLIDAR reported a serial/SDK failure"
      fi
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 10
    fi
    if ! kill -0 "${pid}" 2>/dev/null; then
      kill "${probe_pid}" 2>/dev/null || true
      wait "${probe_pid}" 2>/dev/null || true
      log "ERROR RPLIDAR process pid=${pid} exited before publishing ${SCAN_TOPIC}"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 11
    fi
    if ! kill -0 "${probe_pid}" 2>/dev/null; then
      set +e
      wait "${probe_pid}"
      probe_status=$?
      set -e
      if (( probe_status == 0 )); then
        log "Message received by persistent sensor-data probe: ${SCAN_TOPIC}"
        return 0
      fi
      log "ERROR persistent ${SCAN_TOPIC} probe exited with status=${probe_status}"
      return 13
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
      kill "${probe_pid}" 2>/dev/null || true
      wait "${probe_pid}" 2>/dev/null || true
      log "ERROR no real ${SCAN_TOPIC} message after ${timeout_sec}s; RPLIDAR pid=${pid} remains alive"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${SCAN_TOPIC}; elapsed=${elapsed}s deadline=${timeout_sec}s pid=${pid}"
    fi
    sleep 1
  done
}

wait_for_mavros_connection() {
  local start_ts state elapsed last_progress=-1
  set_component_state MAVROS STARTING "existing_instance_only"
  start_ts="$(date +%s)"
  while true; do
    state="$(timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null || true)"
    if grep -Eq '^[[:space:]]*connected:[[:space:]]*true[[:space:]]*$' <<<"${state}"; then
      set_component_state MAVROS READY "fcu_connected"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= WAIT_TIMEOUT_SEC )); then
      set_component_state MAVROS FAILED "no_connected_state"
      log "ERROR MAVROS did not report FCU connected within ${WAIT_TIMEOUT_SEC}s"
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for existing MAVROS FCU connection; elapsed=${elapsed}s"
    fi
    sleep 1
  done
}

start_rplidar() {
  local attempt pid status=0
  if [[ -n "${EXISTING_RPLIDAR_PID}" ]]; then
    set_component_state RPLIDAR STARTING "reusing_external_pid=${EXISTING_RPLIDAR_PID}"
    set_component_state RAW_SCAN STARTING "deadline=${RPLIDAR_SCAN_WAIT_SEC}s"
    if ! wait_for_rplidar_scan \
      "${EXISTING_RPLIDAR_PID}" "${RPLIDAR_SCAN_WAIT_SEC}" "${RPLIDAR_LOG}"; then
      set_component_state RPLIDAR FAILED "external_driver_has_no_scan"
      set_component_state RAW_SCAN FAILED "external_driver_has_no_scan"
      log "ERROR reused RPLIDAR was not stopped because the launcher does not own it"
      return 1
    fi
    require_publisher_count "${SCAN_TOPIC}" 1
    set_component_state RPLIDAR READY "external_pid=${EXISTING_RPLIDAR_PID}"
    set_component_state RAW_SCAN READY "topic=${SCAN_TOPIC} external_driver=true"
    return 0
  fi
  require_publisher_count "${SCAN_TOPIC}" 0
  for ((attempt=1; attempt<=RPLIDAR_START_RETRIES; attempt++)); do
    set_component_state RPLIDAR STARTING "attempt=${attempt}/${RPLIDAR_START_RETRIES}"
    set_component_state RAW_SCAN STARTING "deadline=${RPLIDAR_SCAN_WAIT_SEC}s"
    start_process "rplidar_attempt_${attempt}" "${RPLIDAR_LOG}" \
      ros2 run rplidar_ros rplidar_composition --ros-args \
        -p channel_type:=serial \
        -p serial_port:="${RPLIDAR_SERIAL_PORT}" \
        -p serial_baudrate:="${RPLIDAR_BAUDRATE}" \
        -p scan_mode:="${RPLIDAR_SCAN_MODE}" \
        -p frame_id:="${RPLIDAR_FRAME_ID}" \
        -p inverted:="${RPLIDAR_INVERTED}" \
        -p angle_compensate:="${RPLIDAR_ANGLE_COMPENSATE}" \
        -p topic_name:="${SCAN_TOPIC#/}"
    pid="${LAST_STARTED_PID}"

    if wait_for_rplidar_scan "${pid}" "${RPLIDAR_SCAN_WAIT_SEC}" "${RPLIDAR_LOG}"; then
      require_publisher_count "${SCAN_TOPIC}" 1
      set_component_state RPLIDAR READY "pid=${pid}"
      set_component_state RAW_SCAN READY "topic=${SCAN_TOPIC}"
      return 0
    else
      status=$?
    fi

    if (( status == 12 )); then
      set_component_state RPLIDAR DEGRADED "owned_driver_alive_but_silent attempt=${attempt}"
      set_component_state RAW_SCAN DEGRADED "no_real_scan_message"
      stop_process_group "${pid}"
      wait "${pid}" 2>/dev/null || true
      if (( attempt < RPLIDAR_START_RETRIES )); then
        log "Launcher-owned RPLIDAR is alive but silent; using the single permitted restart"
        sleep "${RPLIDAR_RETRY_DELAY_SEC}"
        continue
      fi
      break
    fi
    if (( status != 10 )); then
      set_component_state RPLIDAR FAILED "unexpected_process_exit"
      set_component_state RAW_SCAN FAILED "driver_process_exit"
      return 1
    fi
    stop_process_group "${pid}"
    wait "${pid}" 2>/dev/null || true
    if (( attempt < RPLIDAR_START_RETRIES )); then
      log "Actual RPLIDAR failure detected; using the single permitted restart"
      sleep "${RPLIDAR_RETRY_DELAY_SEC}"
    fi
  done
  set_component_state RPLIDAR FAILED "driver_failure"
  set_component_state RAW_SCAN FAILED "driver_failure"
  log "ERROR RPLIDAR failed after the single bounded restart; check scanner motor, USB cable, and 5 V power stability"
  return 1
}

wait_for_transform() {
  local parent="$1"
  local child="$2"
  local timeout_sec="$3"
  shift 3
  local start_ts sample pid elapsed last_progress=-1
  start_ts="$(date +%s)"
  while true; do
    for pid in "$@"; do
      if ! kill -0 "${pid}" 2>/dev/null; then
        log "ERROR process pid=${pid} exited while waiting for TF ${parent} -> ${child}"
        return 10
      fi
    done
    sample="$(timeout 2 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
    if grep -q -- '- Translation:' <<<"${sample}"; then
      log "Transform available: ${parent} -> ${child}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
      log "ERROR missing TF ${parent} -> ${child} after ${timeout_sec}s"
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for TF ${parent} -> ${child}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

wait_for_diagnostic_true_alive() {
  local topic="$1"
  local key="$2"
  local pid="$3"
  local timeout_sec="$4"
  local logfile="$5"
  local start_ts sample elapsed last_progress=-1
  start_ts="$(date +%s)"
  while true; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      log "ERROR process pid=${pid} exited while waiting for ${topic} ${key}=true"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 10
    fi
    sample="$(timeout 3 ros2 topic echo "${topic}" --once 2>/dev/null || true)"
    if grep -A1 -F "key: ${key}" <<<"${sample}" | \
      grep -Eq "value: ['\"]?true['\"]?"; then
      LAST_DIAGNOSTIC_SAMPLE="${sample}"
      log "Diagnostic ready: ${topic} ${key}=true"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
      log "ERROR diagnostic deadline exceeded: ${topic} ${key}=true"
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic} ${key}=true; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

diagnostic_value() {
  local sample="$1"
  local key="$2"
  awk -v wanted="${key}" '
    /key:[[:space:]]*/ {
      candidate=$0
      sub(/^.*key:[[:space:]]*/, "", candidate)
      gsub(/[\047\"]/, "", candidate)
      sub(/[[:space:]]+$/, "", candidate)
      found=(candidate == wanted)
      next
    }
    found && /value:[[:space:]]*/ {
      value=$0
      sub(/^.*value:[[:space:]]*/, "", value)
      gsub(/[\047\"]/, "", value)
      sub(/[[:space:]]+$/, "", value)
      print value
      exit
    }
  ' <<<"${sample}"
}

validate_message_series() {
  local kind="$1"
  local topic="$2"
  local expected_count="$3"
  local expected_bins="$4"
  local producer_pid="$5"
  local timeout_sec="$6"
  if timeout "${timeout_sec}" python3 - "${kind}" "${topic}" "${expected_count}" \
    "${expected_bins}" "${producer_pid}" <<'PY'
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

kind, topic = sys.argv[1], sys.argv[2]
required, expected_bins, producer_pid = map(int, sys.argv[3:6])
if kind == "scan":
    from sensor_msgs.msg import LaserScan as Message
elif kind == "odom":
    from nav_msgs.msg import Odometry as Message
else:
    raise RuntimeError(kind)


class Validator(Node):
    def __init__(self):
        super().__init__("startup_message_series_validator")
        self.consecutive = 0
        self.last_stamp = -1
        self.reference_geometry = None
        self.result = 1
        self.started = time.monotonic()
        self.last_progress = -1
        self.subscription = self.create_subscription(
            Message, topic, self.callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.check_health)

    def check_health(self):
        try:
            os.kill(producer_pid, 0)
        except OSError:
            print(f"producer pid={producer_pid} exited while validating {topic}", flush=True)
            self.result = 10
            rclpy.shutdown()
            return
        bucket = int((time.monotonic() - self.started) // 5)
        if bucket > self.last_progress:
            self.last_progress = bucket
            print(
                f"waiting for {required} coherent {kind} messages on {topic}; "
                f"progress={self.consecutive}/{required} elapsed={bucket * 5}s",
                flush=True)

    def callback(self, msg):
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        valid = stamp > self.last_stamp and len(self.get_publishers_info_by_topic(topic)) == 1
        if kind == "scan":
            geometry = (
                len(msg.ranges), round(float(msg.angle_min), 7),
                round(float(msg.angle_max), 7), round(float(msg.angle_increment), 9),
                msg.header.frame_id)
            if self.reference_geometry is None:
                self.reference_geometry = geometry
            valid = (
                valid and len(msg.ranges) == expected_bins and
                all(math.isfinite(v) for v in (msg.angle_min, msg.angle_max, msg.angle_increment)) and
                msg.angle_increment > 0.0 and msg.header.frame_id != "" and
                geometry == self.reference_geometry)
        if valid:
            self.consecutive += 1
            self.last_stamp = stamp
            print(f"{topic} coherent messages {self.consecutive}/{required}", flush=True)
        else:
            self.consecutive = 0
            if stamp > self.last_stamp:
                self.last_stamp = stamp
        if self.consecutive >= required:
            self.result = 0
            rclpy.shutdown()


rclpy.init()
node = Validator()
try:
    rclpy.spin(node)
finally:
    result = node.result
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
sys.exit(result)
PY
  then
    return 0
  else
    return $?
  fi
}

start_odom_and_tf() {
  set_component_state ODOM_FLATTEN STARTING
  start_process odom_flatten "${ODOM_FLATTEN_LOG}" \
    ros2 run odom_flatten px4_odom_flatten_node --ros-args \
      -p use_sim_time:=false -p odom_topic:="${ODOM_TOPIC}" \
      -p parent_frame:="${ODOM_PARENT_FRAME}" -p child_frame:="${ODOM_CHILD_FRAME}"
  ODOM_FLATTEN_PID="${LAST_STARTED_PID}"

  if wait_for_transform "${ODOM_PARENT_FRAME}" "${ODOM_CHILD_FRAME}" \
    "${WAIT_TIMEOUT_SEC}" "${ODOM_FLATTEN_PID}"; then
    set_component_state ODOM_FLATTEN READY "${ODOM_PARENT_FRAME}->${ODOM_CHILD_FRAME}"
  else
    set_component_state ODOM_FLATTEN FAILED "missing_tf"
    return 1
  fi

  set_component_state STATIC_TF STARTING
  start_process static_tf "${STATIC_TF_LOG}" \
    ros2 run tf2_ros static_transform_publisher \
      --x "${LIDAR_X}" --y "${LIDAR_Y}" --z "${LIDAR_Z}" \
      --roll "${LIDAR_ROLL}" --pitch "${LIDAR_PITCH}" --yaw "${LIDAR_YAW}" \
      --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_FRAME}" \
      --ros-args -p use_sim_time:=false
  STATIC_TF_PID="${LAST_STARTED_PID}"

  if wait_for_transform "${BASE_FRAME}" "${LIDAR_FRAME}" \
    "${WAIT_TIMEOUT_SEC}" "${STATIC_TF_PID}"; then
    set_component_state STATIC_TF READY "${BASE_FRAME}->${LIDAR_FRAME}"
  else
    set_component_state STATIC_TF FAILED "missing_tf"
    return 1
  fi
}

start_scan_audit() {
  set_component_state SCAN_AUDIT STARTING "window=20"
  start_process scan_stream_audit "${SCAN_AUDIT_LOG}" \
    ros2 run obs_avoid laser_scan_stream_audit --ros-args \
      -p input_topic:="${SCAN_TOPIC}" \
      -p diagnostics_topic:="${SCAN_AUDIT_DIAGNOSTICS_TOPIC}" \
      -p csv_path:="${RAW_SCAN_AUDIT_CSV}" \
      -p classification_window_messages:=20 \
      -p full_revolution_min_span_rad:=5.8
  SCAN_AUDIT_PID="${LAST_STARTED_PID}"
  if ! wait_for_diagnostic_true_alive "${SCAN_AUDIT_DIAGNOSTICS_TOPIC}" \
    classification_ready "${SCAN_AUDIT_PID}" "${SCAN_AUDIT_WAIT_SEC}" "${SCAN_AUDIT_LOG}"; then
    set_component_state SCAN_AUDIT FAILED "classification_timeout_or_process_exit"
    return 1
  fi
  RAW_SCAN_CLASSIFICATION="$(diagnostic_value "${LAST_DIAGNOSTIC_SAMPLE}" classification)"
  if [[ "${RAW_SCAN_CLASSIFICATION}" != "FULL_REVOLUTION" &&
    "${RAW_SCAN_CLASSIFICATION}" != "PARTIAL_SECTOR" ]]; then
    set_component_state SCAN_AUDIT FAILED "classification=${RAW_SCAN_CLASSIFICATION:-missing}"
    return 1
  fi
  set_component_state SCAN_AUDIT READY "classification=${RAW_SCAN_CLASSIFICATION}"
  log "Raw scan stream classification: ${RAW_SCAN_CLASSIFICATION}"
}

start_canonicalizer() {
  require_publisher_count "${RF2O_SCAN_TOPIC}" 0
  set_component_state CANONICALIZER STARTING "mode=auto"
  set_component_state CANONICAL_SCAN STARTING "bins=${RF2O_SCAN_BINS}"
  start_process canonicalizer "${CANONICALIZER_LOG}" \
    ros2 run obs_avoid laser_scan_canonicalizer --ros-args \
      -p input_topic:="${SCAN_TOPIC}" -p output_topic:="${RF2O_SCAN_TOPIC}" \
      -p output_frame:="${LIDAR_FRAME}" -p processing_mode:=auto \
      -p output_bins:="${RF2O_SCAN_BINS}" \
      -p output_angle_min:=-3.141592653589793 -p output_angle_max:=3.141592653589793 \
      -p classification_window_messages:=20 -p full_revolution_min_span_rad:=5.8 \
      -p minimum_angular_coverage_ratio:=0.70 -p minimum_finite_return_ratio:=0.05 \
      -p maximum_revolution_duration_sec:=0.50 -p maximum_segment_gap_sec:=0.20 \
      -p maximum_input_messages_per_revolution:=20 -p publish_rate_limit_hz:=12.0 \
      -p diagnostics_rate_hz:=1.0 -p diagnostics_topic:="${RF2O_SCAN_DIAGNOSTICS_TOPIC}" \
      -p health_csv_path:="${CANONICAL_SCAN_HEALTH_CSV}"
  CANONICALIZER_PID="${LAST_STARTED_PID}"
  if ! validate_message_series scan "${RF2O_SCAN_TOPIC}" 5 "${RF2O_SCAN_BINS}" \
    "${CANONICALIZER_PID}" "${CANONICAL_SCAN_WAIT_SEC}"; then
    set_component_state CANONICALIZER DEGRADED "no_five_coherent_outputs"
    set_component_state CANONICAL_SCAN DEGRADED "validation_failed"
    return 1
  fi
  require_publisher_count "${RF2O_SCAN_TOPIC}" 1
  if ! wait_for_diagnostic_true_alive "${RF2O_SCAN_DIAGNOSTICS_TOPIC}" timestamp_monotonic \
    "${CANONICALIZER_PID}" 10 "${CANONICALIZER_LOG}"; then
    set_component_state CANONICALIZER DEGRADED "diagnostics_not_healthy"
    set_component_state CANONICAL_SCAN DEGRADED "diagnostics_not_healthy"
    return 1
  fi
  local coverage
  coverage="$(diagnostic_value "${LAST_DIAGNOSTIC_SAMPLE}" angular_observation_coverage_ratio)"
  if ! awk -v value="${coverage:-0}" 'BEGIN{exit !(value >= 0.70)}'; then
    set_component_state CANONICAL_SCAN DEGRADED "angular_coverage=${coverage:-missing}"
    return 1
  fi
  set_component_state CANONICALIZER READY "mode=$(diagnostic_value "${LAST_DIAGNOSTIC_SAMPLE}" processing_mode)"
  set_component_state CANONICAL_SCAN READY "five_coherent_scans coverage=${coverage}"
}

start_scan_tf_timing_audit() {
  set_component_state SCAN_TF_AUDIT STARTING "exact_scan_stamp_gate"
  start_process scan_tf_timing_audit "${SCAN_TF_TIMING_LOG}" \
    ros2 run obs_avoid scan_tf_timing_audit --ros-args \
      -p use_sim_time:=false -p scan_topic:="${RF2O_SCAN_TOPIC}" \
      -p odom_topic:="${ODOM_TOPIC}" -p odom_frame:="${ODOM_PARENT_FRAME}" \
      -p base_frame:="${BASE_FRAME}" -p laser_frame:="${LIDAR_FRAME}" \
      -p diagnostics_topic:="${SCAN_TF_DIAGNOSTICS_TOPIC}" \
      -p timing_csv_path:="${SCAN_TF_TIMING_CSV}" \
      -p motion_csv_path:="${SCAN_MOTION_CSV}" \
      -p scan_timeout_sec:=0.5 -p odom_timeout_sec:=0.5 \
      -p tf_lookup_timeout_sec:=0.3 -p maximum_allowed_scan_tf_offset_sec:=0.03 \
      -p deskew_yaw_threshold_deg:=2.0 -p deskew_translation_threshold_m:=0.02 \
      -p diagnostics_rate_hz:=1.0
  SCAN_TF_AUDIT_PID="${LAST_STARTED_PID}"
  if ! wait_for_diagnostic_true_alive "${SCAN_TF_DIAGNOSTICS_TOPIC}" timing_valid \
    "${SCAN_TF_AUDIT_PID}" "${SCAN_TF_WAIT_SEC}" "${SCAN_TF_TIMING_LOG}"; then
    set_component_state SCAN_TF_AUDIT FAILED "exact_scan_time_tf_unavailable"
    log "ERROR exact odom->base_footprint and odom->laser_frame TF are not available at scan time"
    return 1
  fi
  set_component_state SCAN_TF_AUDIT READY "exact_scan_time_tf_verified"
}

start_optional_deskewer() {
  if [[ "${USE_DESKEWED_SCAN}" != "1" ]]; then
    set_component_state DESKEWER STOPPED "disabled_by_default"
    set_component_state DESKEWED_SCAN STOPPED "selected_scan=${SELECTED_SCAN_TOPIC}"
    return 0
  fi

  require_publisher_count "${DESKEWED_SCAN_TOPIC}" 0
  set_component_state DESKEWER STARTING "midpoint_reference"
  set_component_state DESKEWED_SCAN STARTING "bins=${RF2O_SCAN_BINS}"
  start_process laser_scan_deskewer "${DESKEWER_LOG}" \
    ros2 run obs_avoid laser_scan_deskewer --ros-args \
      -p use_sim_time:=false -p enabled:=true \
      -p input_topic:="${RF2O_SCAN_TOPIC}" -p output_topic:="${DESKEWED_SCAN_TOPIC}" \
      -p output_frame:="${LIDAR_FRAME}" -p odom_frame:="${ODOM_PARENT_FRAME}" \
      -p reference_time:=midpoint -p diagnostics_topic:="${DESKEW_DIAGNOSTICS_TOPIC}" \
      -p health_csv_path:="${DESKEW_HEALTH_CSV}" -p minimum_valid_tf_ratio:=0.95 \
      -p maximum_tf_lookup_timeout_sec:=0.02 -p output_bins:="${RF2O_SCAN_BINS}" \
      -p output_angle_min:=-3.141592653589793 -p output_angle_max:=3.141592653589793 \
      -p diagnostics_rate_hz:=1.0
  DESKEWER_PID="${LAST_STARTED_PID}"
  if ! validate_message_series scan "${DESKEWED_SCAN_TOPIC}" 5 "${RF2O_SCAN_BINS}" \
    "${DESKEWER_PID}" "${DESKEW_SCAN_WAIT_SEC}"; then
    set_component_state DESKEWER FAILED "no_five_coherent_outputs"
    set_component_state DESKEWED_SCAN FAILED "validation_failed"
    return 1
  fi
  require_publisher_count "${DESKEWED_SCAN_TOPIC}" 1
  if ! wait_for_diagnostic_true_alive "${DESKEW_DIAGNOSTICS_TOPIC}" published \
    "${DESKEWER_PID}" 10 "${DESKEWER_LOG}"; then
    set_component_state DESKEWER FAILED "diagnostics_not_healthy"
    set_component_state DESKEWED_SCAN FAILED "diagnostics_not_healthy"
    return 1
  fi
  set_component_state DESKEWER READY "per_ray_tf_midpoint_reference"
  set_component_state DESKEWED_SCAN READY "five_coherent_scans"
}

start_rf2o_stack() {
  require_publisher_count "${LIDAR_ODOM_TOPIC}" 0
  set_component_state RF2O_MONITOR STARTING "before_rf2o"
  start_process lidar_odom_monitor "${LIDAR_ODOM_MONITOR_LOG}" \
    ros2 run obs_avoid lidar_odom_monitor --ros-args \
      --params-file "${LIDAR_MONITOR_PARAMS_FILE}" \
      -p scan_topic:="${SELECTED_SCAN_TOPIC}" \
      -p scan_diagnostics_topic:="${RF2O_SCAN_DIAGNOSTICS_TOPIC}" \
      -p health_csv_path:="${LIDAR_ODOM_HEALTH_CSV}"
  RF2O_MONITOR_PID="${LAST_STARTED_PID}"
  if wait_for_topic_message_alive "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" \
    "${RF2O_MONITOR_PID}" 10 "${LIDAR_ODOM_MONITOR_LOG}"; then
    set_component_state RF2O_MONITOR DEGRADED "diagnostics_active_waiting_for_rf2o"
  else
    set_component_state RF2O_MONITOR FAILED "process_or_diagnostics_failure"
    return 1
  fi

  require_publisher_count "${RF2O_RAW_ODOM_TOPIC}" 0
  set_component_state RF2O STARTING "scan=${SELECTED_SCAN_TOPIC}"
  start_process rf2o "${RF2O_LOG}" \
    ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
      -r __node:=rf2o_laser_odometry --params-file "${RF2O_PARAMS_FILE}" \
      -p laser_scan_topic:="${SELECTED_SCAN_TOPIC}"
  RF2O_PID="${LAST_STARTED_PID}"
  if validate_message_series odom "${RF2O_RAW_ODOM_TOPIC}" 5 0 \
    "${RF2O_PID}" "${RF2O_ODOM_WAIT_SEC}"; then
    require_publisher_count "${RF2O_RAW_ODOM_TOPIC}" 1
    set_component_state RF2O READY "five_new_raw_odometry_messages"
  else
    if kill -0 "${RF2O_PID}" 2>/dev/null; then
      set_component_state RF2O DEGRADED "no_five_raw_odometry_messages"
    else
      set_component_state RF2O FAILED "process_exited"
    fi
    set_component_state RF2O_MONITOR DEGRADED "rf2o_not_ready"
    return 1
  fi

  if validate_message_series odom "${LIDAR_ODOM_TOPIC}" 5 0 \
    "${RF2O_MONITOR_PID}" "${RF2O_ODOM_WAIT_SEC}"; then
    require_publisher_count "${LIDAR_ODOM_TOPIC}" 1
    set_component_state RF2O_MONITOR READY "five_gated_odometry_messages"
  else
    set_component_state RF2O_MONITOR DEGRADED "monitor_rejected_unstable_rf2o"
  fi
}

start_slam() {
  require_publisher_count /map 0
  set_component_state SLAM STARTING \
    "profile=${SLAM_PROFILE} scan=${SELECTED_SCAN_TOPIC} odom=${ODOM_PARENT_FRAME}"
  set_component_state MAP STARTING "deadline=${WAIT_TIMEOUT_SEC}s"
  start_process slam_toolbox "${SLAM_LOG}" \
    ros2 launch obs_avoid real_slam_timing.launch.py \
      slam_params_file:="${SLAM_PARAMS_FILE}" scan_topic:="${SELECTED_SCAN_TOPIC}" \
      use_sim_time:=false autostart:=true use_lifecycle_manager:=false
  SLAM_PID="${LAST_STARTED_PID}"
  if wait_for_topic_message_alive /map "${SLAM_PID}" "${WAIT_TIMEOUT_SEC}" "${SLAM_LOG}"; then
    require_publisher_count /map 1
    set_component_state SLAM READY "selected_scan_subscription"
    set_component_state MAP READY "messages_available"
  else
    if kill -0 "${SLAM_PID}" 2>/dev/null; then
      set_component_state SLAM DEGRADED "alive_without_map"
      set_component_state MAP DEGRADED "no_message_before_deadline"
    else
      set_component_state SLAM FAILED "process_exited"
      set_component_state MAP FAILED "slam_process_exited"
    fi
    return 1
  fi

  local node_info
  node_info="$(timeout 5 ros2 node info /slam_toolbox 2>/dev/null || true)"
  if ! grep -qF "${SELECTED_SCAN_TOPIC}:" <<<"${node_info}" ||
    grep -Eq '^[[:space:]]*/scan:' <<<"${node_info}" ||
    { [[ "${SELECTED_SCAN_TOPIC}" != "${RF2O_SCAN_TOPIC}" ]] &&
      grep -qF "${RF2O_SCAN_TOPIC}:" <<<"${node_info}"; }; then
    set_component_state SLAM FAILED "incorrect_scan_subscription"
    log "ERROR slam_toolbox is not exclusively subscribed to ${SELECTED_SCAN_TOPIC}"
    printf '%s\n' "${node_info}"
    return 1
  fi
}

start_planner() {
  if [[ "${COMPONENT_STATE[MAP]}" != "READY" ]]; then
    set_component_state PLANNER STOPPED "map_not_ready"
    return 0
  fi
  set_component_state PLANNER STARTING "after_map"
  start_process planner "${PLANNER_LOG}" \
    ros2 run obs_avoid local_planner_mode_a --ros-args \
      --params-file "${PLANNER_PARAMS_FILE}" -p use_sim_time:=false \
      -r /scan_horizontal:="${SELECTED_SCAN_TOPIC}"
  PLANNER_PID="${LAST_STARTED_PID}"
  sleep 2
  if kill -0 "${PLANNER_PID}" 2>/dev/null; then
    set_component_state PLANNER READY "process_alive"
  else
    set_component_state PLANNER FAILED "process_exited"
    return 1
  fi
}

start_px4_bridge() {
  if [[ "${LIDAR_MODE}" != "px4_fusion" || "${START_LIDAR_PX4_BRIDGE}" != "1" ]]; then
    set_component_state PX4_BRIDGE STOPPED "disabled_by_mode"
    log "PX4 bridge disabled; nothing will publish to ${PX4_ODOMETRY_OUT_TOPIC}"
    return 0
  fi
  set_component_state PX4_BRIDGE STARTING "explicit_px4_fusion_mode"
  start_process lidar_odom_px4_bridge "${PX4_BRIDGE_LOG}" \
    ros2 run obs_avoid lidar_odom_px4_bridge --ros-args \
      --params-file "${LIDAR_MONITOR_PARAMS_FILE}" \
      -p health_csv_path:="${PX4_BRIDGE_HEALTH_CSV}"
  PX4_BRIDGE_PID="${LAST_STARTED_PID}"
  if wait_for_topic_message_alive "${LIDAR_PX4_DIAGNOSTICS_TOPIC}" \
    "${PX4_BRIDGE_PID}" 10 "${PX4_BRIDGE_LOG}"; then
    set_component_state PX4_BRIDGE DEGRADED "bridge_gate_active_pending_alignment"
  else
    set_component_state PX4_BRIDGE FAILED "process_or_diagnostics_failure"
    return 1
  fi
}

start_diagnostic_bag() {
  if [[ "${RECORD_LIDAR_DIAGNOSTIC_BAG}" != "1" ]]; then
    return 0
  fi
  start_process lidar_diagnostic_bag "${ROSBAG_LOG}" \
    ros2 bag record -o "${LOG_DIR}/lidar_diagnostic_bag" \
      "${SCAN_TOPIC}" "${RF2O_SCAN_TOPIC}" "${DESKEWED_SCAN_TOPIC}" \
      "${SCAN_AUDIT_DIAGNOSTICS_TOPIC}" "${SCAN_TF_DIAGNOSTICS_TOPIC}" \
      "${DESKEW_DIAGNOSTICS_TOPIC}" /odom_flatten/diagnostics \
      "${RF2O_SCAN_DIAGNOSTICS_TOPIC}" "${RF2O_RAW_ODOM_TOPIC}" "${LIDAR_ODOM_TOPIC}" \
      "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" "${ODOM_TOPIC}" /mavros/imu/data /map /tf /tf_static
}

capture_runtime_snapshot() {
  {
    printf '\n[%s] runtime snapshot\n' "$(timestamp)"
    printf 'lidar_mode=%s\nslam_profile=%s\nraw_scan_topic=%s\nrf2o_scan_topic=%s\nselected_scan_topic=%s\nuse_deskewed_scan=%s\ncanonical_scan_bins=%s\n' \
      "${LIDAR_MODE}" "${SLAM_PROFILE}" "${SCAN_TOPIC}" "${RF2O_SCAN_TOPIC}" \
      "${SELECTED_SCAN_TOPIC}" "${USE_DESKEWED_SCAN}" "${RF2O_SCAN_BINS}"
    printf '\n-- component states --\n'
    local component
    for component in "${COMPONENTS[@]}"; do
      printf '%s_STATE=%s\n' "${component}" "${COMPONENT_STATE[${component}]}"
    done
    printf '\n-- process list --\n'
    pgrep -af 'rplidar|scan_stream_audit|canonicalizer|timing_audit|deskewer|rf2o|slam_toolbox|odom_flatten|lidar_odom' || true
    printf '\n-- topic endpoints --\n'
    ros2 topic info "${SCAN_TOPIC}" -v || true
    ros2 topic info "${RF2O_SCAN_TOPIC}" -v || true
    ros2 topic info "${SELECTED_SCAN_TOPIC}" -v || true
    ros2 topic info "${RF2O_RAW_ODOM_TOPIC}" -v || true
    ros2 topic info "${LIDAR_ODOM_TOPIC}" -v || true
    ros2 topic info /map -v || true
    printf '\n-- scan audit --\n'
    timeout 5 ros2 topic echo "${SCAN_AUDIT_DIAGNOSTICS_TOPIC}" --once || true
    printf '\n-- canonical scan --\n'
    timeout 5 ros2 topic echo "${RF2O_SCAN_DIAGNOSTICS_TOPIC}" --once || true
    printf '\n-- scan/TF timing --\n'
    timeout 5 ros2 topic echo "${SCAN_TF_DIAGNOSTICS_TOPIC}" --once || true
    if [[ "${USE_DESKEWED_SCAN}" == "1" ]]; then
      printf '\n-- deskew --\n'
      timeout 5 ros2 topic echo "${DESKEW_DIAGNOSTICS_TOPIC}" --once || true
    fi
    printf '\n-- RF2O monitor --\n'
    timeout 5 ros2 topic echo "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" --once || true
    printf '\n-- SLAM node --\n'
    ros2 node info /slam_toolbox || true
  } >>"${SYSTEM_SNAPSHOT}" 2>&1
}

process_health_loop() {
  while true; do
    local index pid metrics
    for index in "${!PIDS[@]}"; do
      pid="${PIDS[$index]}"
      metrics="$(ps -p "${pid}" -o %cpu=,%mem=,rss= 2>/dev/null || true)"
      if [[ -n "${metrics}" ]]; then
        read -r cpu memory rss <<<"${metrics}"
        printf '%s,%s,%s,%s,%s,%s,true\n' "$(timestamp)" "${NAMES[$index]}" "${pid}" \
          "${cpu}" "${memory}" "${rss}" >>"${PROCESS_HEALTH_CSV}"
      else
        printf '%s,%s,%s,0,0,0,false\n' "$(timestamp)" "${NAMES[$index]}" "${pid}" \
          >>"${PROCESS_HEALTH_CSV}"
      fi
    done
    sleep 1
  done
}

tf_health_loop() {
  while true; do
    local odom_tf laser_tf map_tf base_x="" base_y="" base_yaw=""
    odom_tf="$(timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 || true)"
    laser_tf="$(timeout 2 ros2 run tf2_ros tf2_echo base_footprint laser_frame 2>&1 || true)"
    map_tf="$(timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 || true)"
    if grep -q -- '- Translation:' <<<"${odom_tf}"; then
      base_x="$(sed -n 's/.*Translation: \[\([^,]*\),.*/\1/p' <<<"${odom_tf}" | head -n 1)"
      base_y="$(sed -n 's/.*Translation: \[[^,]*, \([^,]*\),.*/\1/p' <<<"${odom_tf}" | head -n 1)"
      base_yaw="$(sed -n 's/.*RPY (degree).*\[[^,]*, [^,]*, \([^]]*\)\].*/\1/p' <<<"${odom_tf}" | head -n 1)"
    fi
    printf '%s,%s,%s,%s,%s,%s,%s\n' "$(timestamp)" \
      "$(grep -q -- '- Translation:' <<<"${odom_tf}" && printf true || printf false)" \
      "$(grep -q -- '- Translation:' <<<"${laser_tf}" && printf true || printf false)" \
      "$(grep -q -- '- Translation:' <<<"${map_tf}" && printf true || printf false)" \
      "${base_x}" "${base_y}" "${base_yaw}" >>"${TF_HEALTH_CSV}"
    sleep 2
  done
}

monitor_processes() {
  declare -A reported=()
  while true; do
    local index
    for index in "${!PIDS[@]}"; do
      if [[ -z "${reported[${PIDS[$index]}]:-}" ]] &&
        ! kill -0 "${PIDS[$index]}" 2>/dev/null; then
        reported["${PIDS[$index]}"]=1
        log "Process exited: ${NAMES[$index]} pid=${PIDS[$index]}"
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
  local helper
  for helper in "${PROCESS_MONITOR_PID}" "${HEALTH_MONITOR_PID}" "${TF_MONITOR_PID}" \
    "${SNAPSHOT_PID}"; do
    if [[ -n "${helper}" ]] && kill -0 "${helper}" 2>/dev/null; then
      kill "${helper}" 2>/dev/null
      wait "${helper}" 2>/dev/null
    fi
  done
  if [[ -n "${CONSOLE_PID}" ]] && kill -0 "${CONSOLE_PID}" 2>/dev/null; then
    stop_process_group "${CONSOLE_PID}"
    wait "${CONSOLE_PID}" 2>/dev/null
  fi
  stop_owned_token "${RPLIDAR_PROBE_TOKEN}" "rplidar_scan_probe"
  local index component state
  for ((index=${#PIDS[@]} - 1; index>=0; index--)); do
    stop_owned_token "${TOKENS[$index]}" "${NAMES[$index]}"
  done
  for index in "${!PIDS[@]}"; do
    wait "${PIDS[$index]}" 2>/dev/null || true
  done
  for component in "${COMPONENTS[@]}"; do
    state="${COMPONENT_STATE[${component}]}"
    if [[ "${state}" == "STARTING" || "${state}" == "READY" || "${state}" == "DEGRADED" ]]; then
      set_component_state "${component}" STOPPED "launcher_cleanup"
    fi
  done
  log "MAVROS, FCU, webcam, AprilTag detector, and precision-landing pipeline were not stopped"
  if [[ -n "${EXISTING_RPLIDAR_PID}" ]]; then
    log "Reused external RPLIDAR pid=${EXISTING_RPLIDAR_PID} was not stopped"
  fi
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 128
}

start_command_console() {
  if [[ "${LIDAR_MODE}" != "mapping_only" && "${LIDAR_MODE}" != "px4_fusion" ]]; then
    return 0
  fi
  if [[ "${COMPONENT_STATE[PLANNER]}" != "READY" ]]; then
    log "Command console not started because planner is not ready"
    return 0
  fi
  if [[ ! -r /dev/tty ]]; then
    log "WARNING no controlling TTY; command console not started"
    return 0
  fi
  local console_cmd=(
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
  CONSOLE_TOKEN="${LAUNCH_OWNER_ID}:command_console"
  setsid env REAL_SLAM_OWNER_TOKEN="${CONSOLE_TOKEN}" \
    "${console_cmd[@]}" </dev/tty > >(tee -a "${CONSOLE_LOG}") 2>&1 &
  CONSOLE_PID="$!"
  log "Started command console pid=${CONSOLE_PID}; it sends no command until user input"
}

main() {
  mkdir -p "${LOG_DIR}"
  touch "${MASTER_LOG}" "${SYSTEM_SNAPSHOT}" "${RPLIDAR_LOG}" "${SCAN_AUDIT_LOG}" \
    "${RAW_SCAN_AUDIT_CSV}" "${CANONICALIZER_LOG}" "${CANONICAL_SCAN_HEALTH_CSV}" \
    "${RF2O_LOG}" "${LIDAR_ODOM_MONITOR_LOG}" "${LIDAR_ODOM_HEALTH_CSV}" \
    "${SLAM_LOG}" "${ODOM_FLATTEN_LOG}" "${STATIC_TF_LOG}" "${PLANNER_LOG}" \
    "${CONSOLE_LOG}" "${PX4_BRIDGE_LOG}" "${PX4_BRIDGE_HEALTH_CSV}" "${ROSBAG_LOG}" \
    "${SCAN_TF_TIMING_LOG}" "${SCAN_TF_TIMING_CSV}" "${SCAN_MOTION_CSV}" \
    "${DESKEWER_LOG}" "${DESKEW_HEALTH_CSV}"
  printf 'timestamp,process,pid,cpu_percent,memory_percent,rss_kb,alive\n' >"${PROCESS_HEALTH_CSV}"
  printf 'timestamp,odom_to_base_available,base_to_laser_available,map_to_odom_available,base_x,base_y,base_yaw_deg\n' \
    >"${TF_HEALTH_CSV}"
  exec > >(tee -a "${MASTER_LOG}") 2>&1
  initialize_component_states
  trap cleanup EXIT
  trap 'on_signal INT' INT
  trap 'on_signal TERM' TERM

  require_cmd ros2
  require_cmd timeout
  require_cmd pgrep
  require_cmd setsid
  require_cmd python3
  require_cmd awk
  case "${LIDAR_MODE}" in
    mapping_only|rf2o_validation|px4_fusion) ;;
    *) log "ERROR unsupported LIDAR_MODE=${LIDAR_MODE}"; exit 1 ;;
  esac
  case "${SLAM_PROFILE}" in
    normal|timing_debug) ;;
    *) log "ERROR unsupported SLAM_PROFILE=${SLAM_PROFILE}"; exit 1 ;;
  esac
  case "${USE_DESKEWED_SCAN}" in
    0|1) ;;
    *) log "ERROR USE_DESKEWED_SCAN must be 0 or 1"; exit 1 ;;
  esac
  if [[ "${LIDAR_MODE}" != "px4_fusion" && "${START_LIDAR_PX4_BRIDGE}" != "0" ]]; then
    log "ERROR START_LIDAR_PX4_BRIDGE must remain 0 outside px4_fusion mode"
    exit 1
  fi
  if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "${ROS_SETUP}" ]]; then
    log "ERROR ROS setup is missing"
    exit 1
  fi
  if [[ ! -f "${SLAM_PARAMS_FILE}" || ! -f "${PLANNER_PARAMS_FILE}" ||
    ! -f "${LIDAR_MONITOR_PARAMS_FILE}" || ! -f "${RF2O_PARAMS_FILE}" ]]; then
    log "ERROR required pipeline configuration is missing"
    exit 1
  fi

  set +u
  source /opt/ros/jazzy/setup.bash
  source "${ROS_SETUP}"
  set -u
  {
    printf '[%s] independent LiDAR mapping/RF2O run\n' "$(timestamp)"
    printf 'workspace=%s\nlidar_mode=%s\ngit_branch=%s\ngit_commit=%s\n' \
      "${ROS_WS}" "${LIDAR_MODE}" "$(git -C "${ROS_WS}" branch --show-current)" \
      "$(git -C "${ROS_WS}" rev-parse HEAD)"
    printf 'start_lidar_px4_bridge=%s\npx4_output_topic=%s\nslam_profile=%s\nselected_scan_topic=%s\nuse_deskewed_scan=%s\n' \
      "${START_LIDAR_PX4_BRIDGE}" "${PX4_ODOMETRY_OUT_TOPIC}" "${SLAM_PROFILE}" \
      "${SELECTED_SCAN_TOPIC}" "${USE_DESKEWED_SCAN}"
    printf 'rplidar_port=%s\nrplidar_baudrate=%s\nrplidar_scan_mode=%s\nrplidar_scan_wait_sec=%s\n' \
      "${RPLIDAR_SERIAL_PORT}" "${RPLIDAR_BAUDRATE}" "${RPLIDAR_SCAN_MODE}" \
      "${RPLIDAR_SCAN_WAIT_SEC}"
  } >>"${SYSTEM_SNAPSHOT}"
  record_device rplidar "${RPLIDAR_SERIAL_PORT}"

  log "Mode=${LIDAR_MODE}; existing MAVROS and AprilTag services will not be managed"
  wait_for_mavros_connection
  require_no_duplicate_processes
  if [[ "${LIDAR_MODE}" != "px4_fusion" ]]; then
    require_publisher_count "${PX4_ODOMETRY_OUT_TOPIC}" 0
  fi
  if [[ ! -e "${RPLIDAR_SERIAL_PORT}" || ! -r "${RPLIDAR_SERIAL_PORT}" ||
    ! -w "${RPLIDAR_SERIAL_PORT}" ]]; then
    log "ERROR ${RPLIDAR_SERIAL_PORT} is absent or not readable/writable by $(id -un)"
    exit 1
  fi

  start_diagnostic_bag
  start_odom_and_tf
  start_rplidar
  start_scan_audit
  if ! start_canonicalizer; then
    log "ERROR coherent canonical scans are unavailable; SLAM and RF2O will not start"
    exit 1
  fi
  if ! start_scan_tf_timing_audit; then
    log "ERROR scan-time TF validation failed; SLAM and RF2O will not start"
    exit 1
  fi
  if ! start_optional_deskewer; then
    log "ERROR requested deskewed scan is unhealthy; SLAM and RF2O will not start"
    exit 1
  fi
  log "Selected SLAM/RF2O scan topic: ${SELECTED_SCAN_TOPIC}"

  if [[ "${LIDAR_MODE}" == "mapping_only" ]]; then
    if start_slam; then
      start_planner || true
    fi
    if [[ "${START_RF2O_OBSERVER}" == "1" ]]; then
      start_rf2o_stack || log "DEGRADED_RF2O: mapping remains active"
    else
      set_component_state RF2O STOPPED "optional_observer_disabled"
      set_component_state RF2O_MONITOR STOPPED "optional_observer_disabled"
    fi
  elif [[ "${LIDAR_MODE}" == "rf2o_validation" ]]; then
    start_rf2o_stack || log "DEGRADED_RF2O: validation nodes remain available for diagnostics"
    if [[ "${START_SLAM_IN_RF2O_VALIDATION}" == "1" ]]; then
      start_slam || true
    else
      set_component_state SLAM STOPPED "disabled_in_rf2o_validation"
      set_component_state MAP STOPPED "disabled_in_rf2o_validation"
    fi
    set_component_state PLANNER STOPPED "not_used_in_rf2o_validation"
  else
    start_rf2o_stack || log "DEGRADED_RF2O: PX4 bridge will remain gated"
    start_slam || true
    start_planner || true
  fi
  start_px4_bridge || true

  capture_runtime_snapshot &
  SNAPSHOT_PID="$!"
  process_health_loop &
  HEALTH_MONITOR_PID="$!"
  tf_health_loop &
  TF_MONITOR_PID="$!"
  monitor_processes &
  PROCESS_MONITOR_PID="$!"
  start_command_console

  log "Pipeline startup complete. No arm, mode, movement, or PX4 parameter command was sent."
  log "Runtime logs: ${LOG_DIR}"
  if [[ "${LIDAR_MODE}" != "px4_fusion" ]]; then
    log "PX4 fusion is disabled; ${PX4_ODOMETRY_OUT_TOPIC} has no launcher-owned publisher"
  fi

  if [[ -n "${CONSOLE_PID}" ]]; then
    set +e
    wait "${CONSOLE_PID}"
    console_status=$?
    set -e
    SHUTDOWN_REASON="command console exited with status ${console_status}"
    return "${console_status}"
  fi
  log "Diagnostic mode running; press Ctrl-C to stop launcher-owned processes"
  while true; do
    sleep 5
  done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
