#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091

set -euo pipefail

ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
LIDAR_MODE="${LIDAR_MODE:-px4_fusion}"
START_LIDAR_PX4_BRIDGE="${START_LIDAR_PX4_BRIDGE:-1}"
START_RF2O_OBSERVER="${START_RF2O_OBSERVER:-0}"
START_SLAM_IN_RF2O_VALIDATION="${START_SLAM_IN_RF2O_VALIDATION:-1}"
RECORD_LIDAR_DIAGNOSTIC_BAG="${RECORD_LIDAR_DIAGNOSTIC_BAG:-${RECORD_BAG:-1}}"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
RPLIDAR_START_RETRIES=2
REUSE_EXISTING_RPLIDAR="${REUSE_EXISTING_RPLIDAR:-1}"
RPLIDAR_SCAN_WAIT_SEC="${RPLIDAR_SCAN_WAIT_SEC:-60}"
RPLIDAR_RETRY_DELAY_SEC="${RPLIDAR_RETRY_DELAY_SEC:-2}"
RPLIDAR_SERIAL_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB0}"
RPLIDAR_BAUDRATE="${RPLIDAR_BAUDRATE:-115200}"
RPLIDAR_FRAME_ID="${RPLIDAR_FRAME_ID:-laser_frame}"
RPLIDAR_INVERTED="${RPLIDAR_INVERTED:-false}"
RPLIDAR_ANGLE_COMPENSATE="${RPLIDAR_ANGLE_COMPENSATE:-true}"

SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
SCAN_AUDIT_DIAGNOSTICS_TOPIC="${SCAN_AUDIT_DIAGNOSTICS_TOPIC:-/scan_stream_audit/diagnostics}"
RF2O_SCAN_TOPIC="${RF2O_SCAN_TOPIC:-/scan_rf2o}"
RF2O_SCAN_DIAGNOSTICS_TOPIC="${RF2O_SCAN_DIAGNOSTICS_TOPIC:-/scan_rf2o/diagnostics}"
RF2O_SCAN_BINS="${RF2O_SCAN_BINS:-720}"
SCAN_AUDIT_WAIT_SEC="${SCAN_AUDIT_WAIT_SEC:-30}"
CANONICAL_SCAN_WAIT_SEC="${CANONICAL_SCAN_WAIT_SEC:-45}"
RF2O_MONITOR_DIAG_WAIT_SEC="${RF2O_MONITOR_DIAG_WAIT_SEC:-10}"
RF2O_ODOM_WAIT_SEC="${RF2O_ODOM_WAIT_SEC:-60}"
RF2O_RAW_ODOM_TOPIC="${RF2O_RAW_ODOM_TOPIC:-/lidar/odom_raw}"
LIDAR_ODOM_TOPIC="${LIDAR_ODOM_TOPIC:-/lidar/odom}"
LIDAR_ODOM_DIAGNOSTICS_TOPIC="${LIDAR_ODOM_DIAGNOSTICS_TOPIC:-/lidar_odom/diagnostics}"
PX4_ODOMETRY_OUT_TOPIC="${PX4_ODOMETRY_OUT_TOPIC:-/mavros/odometry/out}"
LIDAR_PX4_DIAGNOSTICS_TOPIC="${LIDAR_PX4_DIAGNOSTICS_TOPIC:-/lidar_odom_px4_bridge/diagnostics}"
PX4_BRIDGE_WAIT_SEC="${PX4_BRIDGE_WAIT_SEC:-60}"

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
ENABLE_SCAN_DESKEW="${ENABLE_SCAN_DESKEW:-true}"
DESKEW_FIXED_FRAME="${DESKEW_FIXED_FRAME:-${ODOM_PARENT_FRAME}}"
DESKEW_STAMP_POLICY="${DESKEW_STAMP_POLICY:-end}"
DESKEW_TIMEOUT_SEC="${DESKEW_TIMEOUT_SEC:-0.35}"

SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar.yaml}"
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

COMPONENTS=(
  MAVROS RPLIDAR RAW_SCAN ODOM_FLATTEN STATIC_TF SCAN_AUDIT CANONICALIZER
  CANONICAL_SCAN RF2O RF2O_MONITOR SLAM MAP PLANNER PX4_BRIDGE
)
PIDS=()
NAMES=()
STARTUP_NOT_READY=()
LAST_STARTED_PID=""
EXISTING_RPLIDAR_PID=""
CONSOLE_PID=""
PROCESS_MONITOR_PID=""
HEALTH_MONITOR_PID=""
TF_MONITOR_PID=""
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

startup_ready_components() {
  local component
  local -a required=(
    MAVROS RPLIDAR RAW_SCAN ODOM_FLATTEN STATIC_TF SCAN_AUDIT
    CANONICALIZER CANONICAL_SCAN SLAM MAP PLANNER
  )
  if [[ "${LIDAR_MODE}" == "px4_fusion" ]]; then
    required+=(RF2O RF2O_MONITOR PX4_BRIDGE)
  elif [[ "${START_RF2O_OBSERVER}" == "1" ]]; then
    required+=(RF2O RF2O_MONITOR)
  fi
  STARTUP_NOT_READY=()
  for component in "${required[@]}"; do
    if [[ "${COMPONENT_STATE[${component}]:-NOT_STARTED}" != "READY" ]]; then
      STARTUP_NOT_READY+=("${component}=${COMPONENT_STATE[${component}]:-NOT_STARTED}")
    fi
  done
  ((${#STARTUP_NOT_READY[@]} == 0))
}

log_startup_banner() {
  local green yellow reset
  green=$'\033[1;32m'
  yellow=$'\033[1;33m'
  reset=$'\033[0m'
  if startup_ready_components; then
    printf '\n%s' "${green}"
    printf '######################################################################\n'
    printf '#                                                                    #\n'
    printf '#                         ALL SYSTEM READY                           #\n'
    printf '#                                                                    #\n'
    printf '#          LiDAR / RF2O / SLAM pipeline startup is complete          #\n'
    printf '#                                                                    #\n'
    printf '######################################################################\n'
    printf '%s\n' "${reset}"
  else
    printf '\n%s' "${yellow}"
    printf '######################################################################\n'
    printf '#                                                                    #\n'
    printf '#                      SYSTEM STARTED - DEGRADED                     #\n'
    printf '#                                                                    #\n'
    printf '#                  Check the not-ready components below              #\n'
    printf '#                                                                    #\n'
    printf '######################################################################\n'
    printf '%s\n' "${reset}"
    log "Not-ready startup components: ${STARTUP_NOT_READY[*]}"
  fi
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
  PIDS+=("${pid}")
  NAMES+=("${name}")
  printf '%s\n' "${pid}" >"${LOG_DIR}/${name}.pid"
  log "Started ${name} pid=${pid}"
}

start_process() {
  local name="$1"
  local logfile="$2"
  shift 2
  setsid "$@" >"${logfile}" 2>&1 &
  LAST_STARTED_PID="$!"
  add_process "${LAST_STARTED_PID}" "${name}"
}

stop_process_group() {
  local pid="$1"
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill -TERM -- "-${pid}" 2>/dev/null || true
  elif kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
  fi
}

wait_for_topic_message_alive() {
  local topic="$1"
  local pid="$2"
  local timeout_sec="$3"
  local logfile="$4"
  local reliability="${5:-best_effort}"
  local timeout_level="${6:-ERROR}"
  local start_ts elapsed last_progress=-1
  start_ts="$(date +%s)"
  while true; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      log "ERROR process pid=${pid} exited before publishing ${topic}"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 10
    fi
    if timeout 2 ros2 topic echo "${topic}" --once \
      --qos-reliability "${reliability}" >/dev/null 2>&1; then
      log "Message received: ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
      log "${timeout_level} no ${topic} message after ${timeout_sec}s; process pid=${pid} remains alive"
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

wait_for_map_message_alive() {
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
    if timeout 3 ros2 topic echo "${topic}" --once \
      --qos-reliability reliable --qos-durability transient_local >/dev/null 2>&1; then
      log "Map message received: ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
      log "ERROR no ${topic} map message after ${timeout_sec}s; process pid=${pid} remains alive"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic} map message; elapsed=${elapsed}s deadline=${timeout_sec}s pid=${pid}"
    fi
    sleep 1
  done
}

rplidar_failure_in_log() {
  grep -Eqi \
    'fatal|cannot open|failed to open|permission denied|device busy|SDK error|bind failed|serial.*error' \
    "$1" 2>/dev/null
}

wait_for_rplidar_scan() {
  local pid="$1"
  local timeout_sec="$2"
  local logfile="$3"
  local start_ts elapsed last_progress=-1
  start_ts="$(date +%s)"
  while true; do
    if rplidar_failure_in_log "${logfile}"; then
      log "ERROR RPLIDAR reported a serial/SDK failure"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 10
    fi
    if ! kill -0 "${pid}" 2>/dev/null; then
      log "ERROR RPLIDAR process pid=${pid} exited before publishing ${SCAN_TOPIC}"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 11
    fi
    if timeout 2 ros2 topic echo "${SCAN_TOPIC}" --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
      log "Message received: ${SCAN_TOPIC}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
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
      -p frame_id:="${RPLIDAR_FRAME_ID}" \
      -p inverted:="${RPLIDAR_INVERTED}" \
      -p angle_compensate:="${RPLIDAR_ANGLE_COMPENSATE}" \
      -p topic_name:="${SCAN_TOPIC#/}" \
      -p use_sim_time:="${USE_SIM_TIME}"
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
      set_component_state RPLIDAR FAILED "alive_but_no_scan_after=${RPLIDAR_SCAN_WAIT_SEC}s"
      set_component_state RAW_SCAN FAILED "no_real_scan_message"
      log "RPLIDAR remains alive and will not be restarted merely for an initialization timeout"
      return 1
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
      -p use_sim_time:="${USE_SIM_TIME}" -p odom_topic:="${ODOM_TOPIC}" \
      -p parent_frame:="${ODOM_PARENT_FRAME}" -p child_frame:="${ODOM_CHILD_FRAME}"
  ODOM_FLATTEN_PID="${LAST_STARTED_PID}"

  set_component_state STATIC_TF STARTING
  start_process static_tf "${STATIC_TF_LOG}" \
    ros2 run tf2_ros static_transform_publisher \
      --x "${LIDAR_X}" --y "${LIDAR_Y}" --z "${LIDAR_Z}" \
      --roll "${LIDAR_ROLL}" --pitch "${LIDAR_PITCH}" --yaw "${LIDAR_YAW}" \
      --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_FRAME}" \
      --ros-args -p use_sim_time:="${USE_SIM_TIME}"
  STATIC_TF_PID="${LAST_STARTED_PID}"

  if wait_for_transform "${ODOM_PARENT_FRAME}" "${ODOM_CHILD_FRAME}" \
    "${WAIT_TIMEOUT_SEC}" "${ODOM_FLATTEN_PID}"; then
    set_component_state ODOM_FLATTEN READY "${ODOM_PARENT_FRAME}->${ODOM_CHILD_FRAME}"
  else
    set_component_state ODOM_FLATTEN FAILED "missing_tf"
    return 1
  fi
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
      -p enable_deskew:="${ENABLE_SCAN_DESKEW}" -p deskew_fixed_frame:="${DESKEW_FIXED_FRAME}" \
      -p deskew_stamp_policy:="${DESKEW_STAMP_POLICY}" \
      -p deskew_timeout_sec:="${DESKEW_TIMEOUT_SEC}" -p use_sim_time:="${USE_SIM_TIME}" \
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

start_rf2o_stack() {
  require_publisher_count "${LIDAR_ODOM_TOPIC}" 0
  set_component_state RF2O_MONITOR STARTING "before_rf2o"
  start_process lidar_odom_monitor "${LIDAR_ODOM_MONITOR_LOG}" \
    ros2 run obs_avoid lidar_odom_monitor --ros-args \
      --params-file "${LIDAR_MONITOR_PARAMS_FILE}" \
      -p health_csv_path:="${LIDAR_ODOM_HEALTH_CSV}" -p use_sim_time:="${USE_SIM_TIME}"
  RF2O_MONITOR_PID="${LAST_STARTED_PID}"
  if wait_for_topic_message_alive "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" \
    "${RF2O_MONITOR_PID}" "${RF2O_MONITOR_DIAG_WAIT_SEC}" "${LIDAR_ODOM_MONITOR_LOG}" reliable WARNING; then
    set_component_state RF2O_MONITOR DEGRADED "diagnostics_active_waiting_for_rf2o"
  else
    local monitor_wait_status=$?
    if [[ "${monitor_wait_status}" -eq 12 ]] && kill -0 "${RF2O_MONITOR_PID}" 2>/dev/null; then
      set_component_state RF2O_MONITOR DEGRADED "diagnostics_pending_starting_rf2o"
      log "WARNING RF2O monitor diagnostics are not ready yet; starting RF2O anyway"
    else
      set_component_state RF2O_MONITOR FAILED "process_or_diagnostics_failure"
      return "${monitor_wait_status}"
    fi
  fi

  require_publisher_count "${RF2O_RAW_ODOM_TOPIC}" 0
  set_component_state RF2O STARTING "scan=${RF2O_SCAN_TOPIC}"
  start_process rf2o "${RF2O_LOG}" \
    ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
      -r __node:=rf2o_laser_odometry --params-file "${RF2O_PARAMS_FILE}" \
      -p use_sim_time:="${USE_SIM_TIME}"
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
  set_component_state SLAM STARTING "scan=${RF2O_SCAN_TOPIC} odom=${ODOM_PARENT_FRAME}"
  set_component_state MAP STARTING "deadline=${WAIT_TIMEOUT_SEC}s"
  start_process slam_toolbox "${SLAM_LOG}" \
    ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:="${SLAM_PARAMS_FILE}" use_sim_time:="${USE_SIM_TIME}"
  SLAM_PID="${LAST_STARTED_PID}"
  if wait_for_map_message_alive /map "${SLAM_PID}" "${WAIT_TIMEOUT_SEC}" "${SLAM_LOG}"; then
    require_publisher_count /map 1
    set_component_state SLAM READY "canonical_scan_subscription"
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
  if ! grep -qF "${RF2O_SCAN_TOPIC}:" <<<"${node_info}" ||
    grep -Eq '^[[:space:]]*/scan:' <<<"${node_info}"; then
    set_component_state SLAM FAILED "incorrect_scan_subscription"
    log "ERROR slam_toolbox is not exclusively subscribed to ${RF2O_SCAN_TOPIC}"
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
      --params-file "${PLANNER_PARAMS_FILE}" -p use_sim_time:="${USE_SIM_TIME}" \
      -r /scan_horizontal:="${RF2O_SCAN_TOPIC}"
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
      -p health_csv_path:="${PX4_BRIDGE_HEALTH_CSV}" -p use_sim_time:="${USE_SIM_TIME}"
  PX4_BRIDGE_PID="${LAST_STARTED_PID}"
  if wait_for_topic_message_alive "${PX4_ODOMETRY_OUT_TOPIC}" \
    "${PX4_BRIDGE_PID}" "${PX4_BRIDGE_WAIT_SEC}" "${PX4_BRIDGE_LOG}" reliable WARNING; then
    set_component_state PX4_BRIDGE READY "publishing=${PX4_ODOMETRY_OUT_TOPIC}"
  else
    local bridge_wait_status=$?
    if [[ "${bridge_wait_status}" -eq 12 ]] && kill -0 "${PX4_BRIDGE_PID}" 2>/dev/null; then
      if timeout 3 ros2 topic echo "${LIDAR_PX4_DIAGNOSTICS_TOPIC}" --once \
        --qos-reliability reliable >/dev/null 2>&1; then
        set_component_state PX4_BRIDGE DEGRADED "diagnostics_active_no_px4_output_before_deadline"
      else
        set_component_state PX4_BRIDGE DEGRADED "process_alive_no_px4_output_before_deadline"
      fi
      log "WARNING PX4 bridge did not publish ${PX4_ODOMETRY_OUT_TOPIC} within ${PX4_BRIDGE_WAIT_SEC}s; continuing with bridge process alive"
      return 0
    fi
    set_component_state PX4_BRIDGE FAILED "process_or_diagnostics_failure"
    return "${bridge_wait_status}"
  fi
}

start_diagnostic_bag() {
  if [[ "${RECORD_LIDAR_DIAGNOSTIC_BAG}" != "1" ]]; then
    return 0
  fi
  start_process lidar_diagnostic_bag "${ROSBAG_LOG}" \
    ros2 bag record -o "${LOG_DIR}/lidar_diagnostic_bag" \
      "${SCAN_TOPIC}" "${RF2O_SCAN_TOPIC}" "${SCAN_AUDIT_DIAGNOSTICS_TOPIC}" \
      "${RF2O_SCAN_DIAGNOSTICS_TOPIC}" "${RF2O_RAW_ODOM_TOPIC}" "${LIDAR_ODOM_TOPIC}" \
      "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" "${ODOM_TOPIC}" /mavros/imu/data /map /tf /tf_static
}

capture_runtime_snapshot() {
  {
    printf '\n[%s] runtime snapshot\n' "$(timestamp)"
    printf 'lidar_mode=%s\nraw_scan_topic=%s\nrf2o_scan_topic=%s\ncanonical_scan_bins=%s\n' \
      "${LIDAR_MODE}" "${SCAN_TOPIC}" "${RF2O_SCAN_TOPIC}" "${RF2O_SCAN_BINS}"
    printf 'use_sim_time=%s\nenable_scan_deskew=%s\ndeskew_fixed_frame=%s\ndeskew_stamp_policy=%s\n' \
      "${USE_SIM_TIME}" "${ENABLE_SCAN_DESKEW}" "${DESKEW_FIXED_FRAME}" "${DESKEW_STAMP_POLICY}"
    printf '\n-- component states --\n'
    local component
    for component in "${COMPONENTS[@]}"; do
      printf '%s_STATE=%s\n' "${component}" "${COMPONENT_STATE[${component}]}"
    done
    printf '\n-- process list --\n'
    pgrep -af 'rplidar|scan_stream_audit|canonicalizer|rf2o|slam_toolbox|odom_flatten|lidar_odom' || true
    printf '\n-- topic endpoints --\n'
    ros2 topic info "${SCAN_TOPIC}" -v || true
    ros2 topic info "${RF2O_SCAN_TOPIC}" -v || true
    ros2 topic info "${RF2O_RAW_ODOM_TOPIC}" -v || true
    ros2 topic info "${LIDAR_ODOM_TOPIC}" -v || true
    ros2 topic info /map -v || true
    printf '\n-- scan audit --\n'
    timeout 5 ros2 topic echo "${SCAN_AUDIT_DIAGNOSTICS_TOPIC}" --once || true
    printf '\n-- canonical scan --\n'
    timeout 5 ros2 topic echo "${RF2O_SCAN_DIAGNOSTICS_TOPIC}" --once || true
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
  for helper in "${PROCESS_MONITOR_PID}" "${HEALTH_MONITOR_PID}" "${TF_MONITOR_PID}"; do
    if [[ -n "${helper}" ]] && kill -0 "${helper}" 2>/dev/null; then
      kill "${helper}" 2>/dev/null
      wait "${helper}" 2>/dev/null
    fi
  done
  if [[ -n "${CONSOLE_PID}" ]] && kill -0 "${CONSOLE_PID}" 2>/dev/null; then
    stop_process_group "${CONSOLE_PID}"
    wait "${CONSOLE_PID}" 2>/dev/null
  fi
  local index pid component state
  for ((index=${#PIDS[@]} - 1; index>=0; index--)); do
    pid="${PIDS[$index]}"
    if kill -0 -- "-${pid}" 2>/dev/null || kill -0 "${pid}" 2>/dev/null; then
      log "Stopping launcher-owned process only: ${NAMES[$index]} pid=${pid}"
      stop_process_group "${pid}"
    fi
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
    -p use_sim_time:="${USE_SIM_TIME}"
  )
  if [[ -n "${PRECLAND_MODE}" ]]; then
    console_cmd+=(-p precland_mode:="${PRECLAND_MODE}")
  fi
  setsid "${console_cmd[@]}" </dev/tty > >(tee -a "${CONSOLE_LOG}") 2>&1 &
  CONSOLE_PID="$!"
  log "Started command console pid=${CONSOLE_PID}; it sends no command until user input"
}

main() {
  mkdir -p "${LOG_DIR}"
  touch "${MASTER_LOG}" "${SYSTEM_SNAPSHOT}" "${RPLIDAR_LOG}" "${SCAN_AUDIT_LOG}" \
    "${RAW_SCAN_AUDIT_CSV}" "${CANONICALIZER_LOG}" "${CANONICAL_SCAN_HEALTH_CSV}" \
    "${RF2O_LOG}" "${LIDAR_ODOM_MONITOR_LOG}" "${LIDAR_ODOM_HEALTH_CSV}" \
    "${SLAM_LOG}" "${ODOM_FLATTEN_LOG}" "${STATIC_TF_LOG}" "${PLANNER_LOG}" \
    "${CONSOLE_LOG}" "${PX4_BRIDGE_LOG}" "${PX4_BRIDGE_HEALTH_CSV}" "${ROSBAG_LOG}"
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
  if [[ "${LIDAR_MODE}" == "px4_fusion" && "${START_LIDAR_PX4_BRIDGE}" != "1" ]]; then
    log "WARNING LIDAR_MODE=px4_fusion forces START_LIDAR_PX4_BRIDGE=1"
    START_LIDAR_PX4_BRIDGE="1"
  fi
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
    printf 'use_sim_time=%s\nenable_scan_deskew=%s\ndeskew_fixed_frame=%s\ndeskew_stamp_policy=%s\n' \
      "${USE_SIM_TIME}" "${ENABLE_SCAN_DESKEW}" "${DESKEW_FIXED_FRAME}" "${DESKEW_STAMP_POLICY}"
    printf 'start_lidar_px4_bridge=%s\npx4_output_topic=%s\npx4_bridge_wait_sec=%s\n' \
      "${START_LIDAR_PX4_BRIDGE}" "${PX4_ODOMETRY_OUT_TOPIC}" "${PX4_BRIDGE_WAIT_SEC}"
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
  start_rplidar
  start_odom_and_tf
  start_scan_audit
  if ! start_canonicalizer; then
    log "ERROR coherent canonical scans are unavailable; SLAM and RF2O will not start"
    exit 1
  fi

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
  if [[ "${LIDAR_MODE}" == "px4_fusion" ]]; then
    start_px4_bridge
  else
    start_px4_bridge || true
  fi

  capture_runtime_snapshot &
  add_process "$!" runtime_snapshot
  process_health_loop &
  HEALTH_MONITOR_PID="$!"
  tf_health_loop &
  TF_MONITOR_PID="$!"
  monitor_processes &
  PROCESS_MONITOR_PID="$!"
  start_command_console

  log_startup_banner
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

main "$@"
