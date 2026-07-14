#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091

set -euo pipefail

ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
RPLIDAR_START_RETRIES=2
RPLIDAR_SCAN_WAIT_SEC="${RPLIDAR_SCAN_WAIT_SEC:-60}"
RPLIDAR_RETRY_DELAY_SEC="${RPLIDAR_RETRY_DELAY_SEC:-2}"
RPLIDAR_SERIAL_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB0}"
RPLIDAR_BAUDRATE="${RPLIDAR_BAUDRATE:-115200}"
RPLIDAR_FRAME_ID="${RPLIDAR_FRAME_ID:-laser_frame}"
RPLIDAR_INVERTED="${RPLIDAR_INVERTED:-false}"
RPLIDAR_ANGLE_COMPENSATE="${RPLIDAR_ANGLE_COMPENSATE:-true}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
RF2O_SCAN_TOPIC="${RF2O_SCAN_TOPIC:-/scan_rf2o}"
RF2O_SCAN_DIAGNOSTICS_TOPIC="${RF2O_SCAN_DIAGNOSTICS_TOPIC:-/scan_rf2o/diagnostics}"
RF2O_SCAN_BINS="${RF2O_SCAN_BINS:-720}"
CANONICAL_SCAN_WAIT_SEC="${CANONICAL_SCAN_WAIT_SEC:-30}"
RF2O_ODOM_WAIT_SEC="${RF2O_ODOM_WAIT_SEC:-30}"
START_LIDAR_PX4_BRIDGE="${START_LIDAR_PX4_BRIDGE:-1}"
PX4_ODOMETRY_OUT_TOPIC="${PX4_ODOMETRY_OUT_TOPIC:-/mavros/odometry/out}"
RF2O_RAW_ODOM_TOPIC="${RF2O_RAW_ODOM_TOPIC:-/lidar/odom_raw}"
LIDAR_ODOM_TOPIC="${LIDAR_ODOM_TOPIC:-/lidar/odom}"
RF2O_ODOM_FRAME="${RF2O_ODOM_FRAME:-lidar_odom}"
RF2O_BASE_FRAME="${RF2O_BASE_FRAME:-base_footprint}"
RF2O_RATE_HZ="${RF2O_RATE_HZ:-10.0}"
LIDAR_ODOM_DIAGNOSTICS_TOPIC="${LIDAR_ODOM_DIAGNOSTICS_TOPIC:-/lidar_odom/diagnostics}"
LIDAR_PX4_DIAGNOSTICS_TOPIC="${LIDAR_PX4_DIAGNOSTICS_TOPIC:-/lidar_odom_px4_bridge/diagnostics}"
RECORD_BAG="${RECORD_BAG:-0}"

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

SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar.yaml}"
PLANNER_PARAMS_FILE="${PLANNER_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/local_planner_mode_a_real_safe.yaml}"
LIDAR_BRIDGE_PARAMS_FILE="${LIDAR_BRIDGE_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/lidar_odom_px4_bridge.yaml}"
RF2O_PARAMS_FILE="${RF2O_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/rf2o_real_a1m8.yaml}"
PX4_EKF_CHECK_SCRIPT="${PX4_EKF_CHECK_SCRIPT:-${ROS_WS}/src/obs_avoid/scripts/check_px4_lidar_ekf.sh}"
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
CANONICALIZER_LOG="${LOG_DIR}/laser_scan_canonicalizer.log"
RF2O_LOG="${LOG_DIR}/rf2o.log"
LIDAR_ODOM_MONITOR_LOG="${LOG_DIR}/lidar_odom_monitor.log"
LIDAR_PX4_BRIDGE_LOG="${LOG_DIR}/lidar_px4_bridge.log"
LIDAR_PX4_HEALTH_CSV="${LOG_DIR}/lidar_px4_bridge_health.csv"
PX4_EKF_FUSION_SNAPSHOT="${LOG_DIR}/px4_ekf_fusion_snapshot.txt"
ROSBAG_LOG="${LOG_DIR}/rosbag.log"
PLANNER_LOG="${LOG_DIR}/planner.log"
CONSOLE_LOG="${LOG_DIR}/command_console.log"
SYSTEM_SNAPSHOT="${LOG_DIR}/system_snapshot.txt"

PIDS=()
NAMES=()
LAST_STARTED_PID=""
MONITOR_PID=""
CONSOLE_PID=""
RPLIDAR_PID=""
SHUTDOWN_REASON="normal console exit"
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
  local now_epoch elapsed old_state start_epoch
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
  for component in RPLIDAR RAW_SCAN TF CANONICAL_SCAN RF2O MONITOR PX4_BRIDGE SLAM PLANNER; do
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

wait_for_topic() {
  local topic="$1"
  local start_ts elapsed last_progress=-1
  start_ts="$(date +%s)"
  while true; do
    if ros2 topic list 2>/dev/null | grep -Fx "${topic}" >/dev/null; then
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

wait_for_message() {
  local topic="$1"
  wait_for_topic "${topic}"
  if ! timeout "${WAIT_TIMEOUT_SEC}" ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    log "ERROR topic exists but no message arrived: ${topic}"
    return 1
  fi
  log "Message received: ${topic}"
}

wait_for_diagnostic_true() {
  local topic="$1"
  local key="$2"
  local start_ts sample
  start_ts="$(date +%s)"
  while true; do
    sample="$(timeout 3 ros2 topic echo "${topic}" --once 2>/dev/null || true)"
    if grep -A1 -F "key: ${key}" <<<"${sample}" | grep -Eq "value: ['\"]?true['\"]?"; then
      log "Diagnostic ready: ${topic} ${key}=true"
      return 0
    fi
    if (( $(date +%s) - start_ts >= WAIT_TIMEOUT_SEC )); then
      log "ERROR timed out waiting for diagnostic: ${topic} ${key}=true"
      return 1
    fi
    sleep 1
  done
}

require_vehicle_disarmed() {
  local state
  state="$(timeout 5 ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  if grep -Eq '^[[:space:]]*armed:[[:space:]]*true[[:space:]]*$' <<<"${state}"; then
    log "WARNING vehicle is armed; LiDAR/PX4 frame alignment is deferred"
    return 1
  fi
  if ! grep -Eq '^[[:space:]]*armed:[[:space:]]*false[[:space:]]*$' <<<"${state}"; then
    log "WARNING could not confirm that the vehicle is disarmed; frame alignment is deferred"
    log "Last /mavros/state sample follows:"
    printf '%s\n' "${state}"
    return 1
  fi
  log "Confirmed vehicle disarmed before LiDAR/PX4 alignment"
}

wait_for_service() {
  local service="$1"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | grep -Fx "${service}" >/dev/null; then
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
    if grep -Eq '^[[:space:]]*connected:[[:space:]]*true[[:space:]]*$' <<<"${state}"; then
      log "Existing MAVROS reports FCU connected"
      return 0
    fi
    if (( $(date +%s) - start_ts >= WAIT_TIMEOUT_SEC )); then
      log "ERROR MAVROS topic exists but FCU did not report connected"
      log "Last /mavros/state sample follows:"
      printf '%s\n' "${state}"
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
  LAST_STARTED_PID="$!"
  add_process "${LAST_STARTED_PID}" "${name}"
}

wait_for_process_message() {
  local topic="$1"
  local pid="$2"
  local timeout_sec="$3"
  local logfile="$4"
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      log "ERROR process exited before publishing ${topic}; pid=${pid}"
      tail -n 30 "${logfile}" 2>/dev/null || true
      return 10
    fi
    if grep -Eqi 'fatal|health[^[:alnum:]]*error|SDK[^[:alnum:]]*error|cannot open|failed to open|permission denied|device busy' \
      "${logfile}" 2>/dev/null; then
      log "ERROR driver reported a fatal LiDAR/SDK/serial condition"
      tail -n 30 "${logfile}" 2>/dev/null || true
      return 11
    fi
    if timeout 2 ros2 topic echo "${topic}" --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
      log "Message received: ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
      log "ERROR no ${topic} message within ${timeout_sec}s; pid=${pid} is still running"
      tail -n 30 "${logfile}" 2>/dev/null || true
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for first ${topic}; elapsed=${elapsed}s deadline=${timeout_sec}s pid=${pid}"
    fi
  done
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
      log "ERROR process exited before publishing ${topic}; pid=${pid}"
      tail -n 30 "${logfile}" 2>/dev/null || true
      return 10
    fi
    if timeout 2 ros2 topic echo "${topic}" --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
      log "Message received: ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start_ts ))
    if (( elapsed >= timeout_sec )); then
      log "WARNING no ${topic} message within ${timeout_sec}s; pid=${pid} remains alive"
      tail -n 30 "${logfile}" 2>/dev/null || true
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic}; elapsed=${elapsed}s deadline=${timeout_sec}s pid=${pid}"
    fi
    sleep 1
  done
}

start_rplidar_with_retry() {
  local attempt pid wait_status
  for ((attempt=1; attempt<=RPLIDAR_START_RETRIES; attempt++)); do
    set_component_state RPLIDAR STARTING "attempt=${attempt}/${RPLIDAR_START_RETRIES}"
    set_component_state RAW_SCAN STARTING "deadline=${RPLIDAR_SCAN_WAIT_SEC}s"
    log "Starting RPLIDAR attempt ${attempt}/${RPLIDAR_START_RETRIES}"
    start_process "rplidar_attempt_${attempt}" "${RPLIDAR_LOG}" \
      ros2 run rplidar_ros rplidar_composition --ros-args \
        -p channel_type:=serial \
        -p serial_port:="${RPLIDAR_SERIAL_PORT}" \
        -p serial_baudrate:="${RPLIDAR_BAUDRATE}" \
        -p frame_id:="${RPLIDAR_FRAME_ID}" \
        -p inverted:="${RPLIDAR_INVERTED}" \
        -p angle_compensate:="${RPLIDAR_ANGLE_COMPENSATE}" \
        -p topic_name:="${SCAN_TOPIC#/}"
    pid="${LAST_STARTED_PID}"

    if wait_for_process_message \
      "${SCAN_TOPIC}" "${pid}" "${RPLIDAR_SCAN_WAIT_SEC}" "${RPLIDAR_LOG}"; then
      RPLIDAR_PID="${pid}"
      set_component_state RPLIDAR READY "pid=${pid}"
      set_component_state RAW_SCAN READY "topic=${SCAN_TOPIC}"
      return 0
    else
      wait_status=$?
    fi

    if (( wait_status == 12 )); then
      RPLIDAR_PID="${pid}"
      set_component_state RAW_SCAN DEGRADED "alive_but_no_scan_after=${RPLIDAR_SCAN_WAIT_SEC}s"
      set_component_state RPLIDAR DEGRADED "driver_alive_not_restarted"
      log "RPLIDAR is alive and still initializing; it will not be restarted merely for this timeout"
      return 0
    fi

    if kill -0 "${pid}" 2>/dev/null; then
      log "Stopping failed launcher-owned RPLIDAR pid=${pid} before the single permitted restart"
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
    if (( attempt < RPLIDAR_START_RETRIES )); then
      sleep "${RPLIDAR_RETRY_DELAY_SEC}"
    fi
  done

  set_component_state RPLIDAR FAILED "actual_driver_failure"
  set_component_state RAW_SCAN FAILED "no_scan"
  log "ERROR RPLIDAR produced no scan after ${RPLIDAR_START_RETRIES} attempts"
  log "Check USB power, cable, motor rotation, permissions, and whether another process owns ${RPLIDAR_SERIAL_PORT}"
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
      log "ERROR missing essential TF ${parent} -> ${child} after ${timeout_sec}s"
      printf '%s\n' "${sample}"
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for TF ${parent} -> ${child}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
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
    raise RuntimeError(f"unsupported message kind: {kind}")


class Validator(Node):
    def __init__(self):
        super().__init__("startup_message_series_validator")
        self.consecutive = 0
        self.last_stamp = -1
        self.result = 1
        self.started = time.monotonic()
        self.last_progress = -1
        self.subscription = self.create_subscription(
            Message, topic, self.callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.check_process)

    def check_process(self):
        try:
            os.kill(producer_pid, 0)
        except OSError:
            print(f"producer pid={producer_pid} exited while validating {topic}", flush=True)
            self.result = 10
            rclpy.shutdown()
            return
        elapsed_bucket = int((time.monotonic() - self.started) // 5)
        if elapsed_bucket > self.last_progress:
            self.last_progress = elapsed_bucket
            print(
                f"waiting for {required} consecutive {kind} messages on {topic}; "
                f"progress={self.consecutive}/{required} elapsed={elapsed_bucket * 5}s",
                flush=True)

    def callback(self, msg):
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        valid = stamp > self.last_stamp
        if kind == "scan":
            valid = valid and len(msg.ranges) == expected_bins
        if valid:
            self.consecutive += 1
            self.last_stamp = stamp
            print(
                f"{topic} validation progress {self.consecutive}/{required} "
                f"stamp={stamp}" +
                (f" ranges={len(msg.ranges)}" if kind == "scan" else ""),
                flush=True)
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

diagnostic_key_true_once() {
  local topic="$1"
  local key="$2"
  local sample
  sample="$(timeout 3 ros2 topic echo "${topic}" --once 2>/dev/null || true)"
  grep -A1 -F "key: ${key}" <<<"${sample}" | grep -Eq "value: ['\"]?true['\"]?"
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
  local component current_state
  for component in RPLIDAR RAW_SCAN TF CANONICAL_SCAN RF2O MONITOR PX4_BRIDGE SLAM PLANNER; do
    current_state="${COMPONENT_STATE[${component}]:-NOT_STARTED}"
    if [[ "${current_state}" == "STARTING" || "${current_state}" == "READY" ||
      "${current_state}" == "DEGRADED" ]]; then
      set_component_state "${component}" STOPPED "launcher_cleanup"
    fi
  done
  log "MAVROS, FCU, webcam, AprilTag detector, and precision-landing pipeline were not stopped"
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 128
}

capture_runtime_snapshot() {
  local canonical_diagnostics canonical_rate_output rf2o_rate_output
  local raw_length_min raw_length_max canonical_length canonical_rate rf2o_rate
  canonical_diagnostics="$(timeout 5 ros2 topic echo \
    "${RF2O_SCAN_DIAGNOSTICS_TOPIC}" --once 2>/dev/null || true)"
  raw_length_min="$(awk '/key: raw_scan_length_min/{found=1; next} found && /value:/{gsub(/['\''\"]/, "", $2); print $2; exit}' \
    <<<"${canonical_diagnostics}")"
  raw_length_max="$(awk '/key: raw_scan_length_max/{found=1; next} found && /value:/{gsub(/['\''\"]/, "", $2); print $2; exit}' \
    <<<"${canonical_diagnostics}")"
  canonical_length="$(awk '/key: output_array_length/{found=1; next} found && /value:/{gsub(/['\''\"]/, "", $2); print $2; exit}' \
    <<<"${canonical_diagnostics}")"
  canonical_rate_output="$(timeout 8 ros2 topic hz "${RF2O_SCAN_TOPIC}" 2>&1 || true)"
  canonical_rate="$(sed -n 's/^average rate: //p' <<<"${canonical_rate_output}" | tail -n 1)"
  rf2o_rate_output="$(timeout 8 ros2 topic hz "${RF2O_RAW_ODOM_TOPIC}" 2>&1 || true)"
  rf2o_rate="$(sed -n 's/^average rate: //p' <<<"${rf2o_rate_output}" | tail -n 1)"
  {
    printf '\n[%s] ROS graph after startup\n' "$(timestamp)"
    printf 'raw_scan_topic=%s\n' "${SCAN_TOPIC}"
    printf 'rf2o_scan_topic=%s\n' "${RF2O_SCAN_TOPIC}"
    printf 'canonical_scan_bins=%s\n' "${RF2O_SCAN_BINS}"
    printf 'raw_scan_length_min=%s\n' "${raw_length_min:-unavailable}"
    printf 'raw_scan_length_max=%s\n' "${raw_length_max:-unavailable}"
    printf 'canonical_scan_length=%s\n' "${canonical_length:-unavailable}"
    printf 'canonical_scan_rate=%s\n' "${canonical_rate:-unavailable}"
    printf 'rf2o_odom_rate=%s\n' "${rf2o_rate:-unavailable}"
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
    printf '\n-- canonical scan diagnostics and metrics --\n'
    printf '%s\n' "${canonical_diagnostics}"
    printf '\n-- canonical scan rate --\n'
    printf '%s\n' "${canonical_rate_output}"
    printf '\n-- odometry rate --\n'
    timeout 8 ros2 topic hz /mavros/local_position/odom || true
    printf '\n-- RF2O raw odometry rate --\n'
    printf '%s\n' "${rf2o_rate_output}"
    printf '\n-- monitored LiDAR odometry rate --\n'
    timeout 8 ros2 topic hz "${LIDAR_ODOM_TOPIC}" || true
    if [[ "${START_LIDAR_PX4_BRIDGE}" == "1" ]]; then
      printf '\n-- MAVROS external odometry rate --\n'
      timeout 8 ros2 topic hz "${PX4_ODOMETRY_OUT_TOPIC}" || true
      printf '\n-- LiDAR/PX4 bridge diagnostics --\n'
      timeout 5 ros2 topic echo "${LIDAR_PX4_DIAGNOSTICS_TOPIC}" --once || true
    fi
    printf '\n-- TF map to odom --\n'
    timeout 5 ros2 run tf2_ros tf2_echo map odom || true
    printf '\n-- TF odom to base_footprint --\n'
    timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint || true
    printf '\n-- TF base_footprint to laser_frame --\n'
    timeout 5 ros2 run tf2_ros tf2_echo base_footprint laser_frame || true
  } >>"${SYSTEM_SNAPSHOT}" 2>&1
}

main() {
  mkdir -p "${LOG_DIR}"
  touch "${MASTER_LOG}" "${RPLIDAR_LOG}" "${SLAM_LOG}" "${ODOM_FLATTEN_LOG}" \
    "${STATIC_TF_LOG}" "${CANONICALIZER_LOG}" "${RF2O_LOG}" "${LIDAR_ODOM_MONITOR_LOG}" \
    "${LIDAR_PX4_BRIDGE_LOG}" "${LIDAR_PX4_HEALTH_CSV}" \
    "${PX4_EKF_FUSION_SNAPSHOT}" "${ROSBAG_LOG}" "${PLANNER_LOG}" \
    "${CONSOLE_LOG}" "${SYSTEM_SNAPSHOT}"
  exec > >(tee -a "${MASTER_LOG}") 2>&1

  initialize_component_states

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
  if [[ ! -f "${SLAM_PARAMS_FILE}" || ! -f "${PLANNER_PARAMS_FILE}" ||
    ! -f "${LIDAR_BRIDGE_PARAMS_FILE}" || ! -f "${RF2O_PARAMS_FILE}" ||
    ! -x "${PX4_EKF_CHECK_SCRIPT}" ]]; then
    log "ERROR SLAM, planner, LiDAR/RF2O configuration, or EKF checker is missing"
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
    printf 'rplidar_path=%s baud=%s topic=%s frame=%s inverted=%s angle_compensate=%s\n' \
      "${RPLIDAR_SERIAL_PORT}" "${RPLIDAR_BAUDRATE}" "${SCAN_TOPIC}" "${RPLIDAR_FRAME_ID}" \
      "${RPLIDAR_INVERTED}" "${RPLIDAR_ANGLE_COMPENSATE}"
    printf 'precland_mode=%s\n' "${PRECLAND_MODE:-NOT_CONFIGURED}"
    printf 'rf2o_upstream_commit=b38c68e46387b98845ecbfeb6660292f967a00d3\n'
    printf 'rf2o_raw_topic=%s lidar_odom_topic=%s rf2o_frame=%s rf2o_base=%s\n' \
      "${RF2O_RAW_ODOM_TOPIC}" "${LIDAR_ODOM_TOPIC}" "${RF2O_ODOM_FRAME}" "${RF2O_BASE_FRAME}"
    printf 'start_lidar_px4_bridge=%s px4_odometry_out_topic=%s\n' \
      "${START_LIDAR_PX4_BRIDGE}" "${PX4_ODOMETRY_OUT_TOPIC}"
    printf 'raw_scan_topic=%s\n' "${SCAN_TOPIC}"
    printf 'rf2o_scan_topic=%s\n' "${RF2O_SCAN_TOPIC}"
    printf 'canonical_scan_bins=%s\n' "${RF2O_SCAN_BINS}"
  } >>"${SYSTEM_SNAPSHOT}"
  record_device "rplidar" "${RPLIDAR_SERIAL_PORT}"

  log "Reusing existing MAVROS and boot-time AprilTag pipeline; this launcher will not manage them"
  log "Selected LiDAR path: ${RPLIDAR_SERIAL_PORT} -> $(readlink -f "${RPLIDAR_SERIAL_PORT}" 2>/dev/null || printf 'not-present')"

  wait_for_topic /mavros/state
  wait_for_topic /mavros/local_position/pose
  wait_for_message /mavros/local_position/odom
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

  if ! start_rplidar_with_retry; then
    exit 1
  fi

  set_component_state TF STARTING "odom_and_laser_chain"
  start_process odom_flatten "${ODOM_FLATTEN_LOG}" \
    ros2 run odom_flatten px4_odom_flatten_node --ros-args \
      -p use_sim_time:=false \
      -p odom_topic:="${ODOM_TOPIC}" \
      -p parent_frame:="${ODOM_PARENT_FRAME}" \
      -p child_frame:="${ODOM_CHILD_FRAME}"
  odom_flatten_pid="${LAST_STARTED_PID}"

  start_process static_tf "${STATIC_TF_LOG}" \
    ros2 run tf2_ros static_transform_publisher \
      --x "${LIDAR_X}" --y "${LIDAR_Y}" --z "${LIDAR_Z}" \
      --roll "${LIDAR_ROLL}" --pitch "${LIDAR_PITCH}" --yaw "${LIDAR_YAW}" \
      --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_FRAME}" \
      --ros-args -p use_sim_time:=false
  static_tf_pid="${LAST_STARTED_PID}"

  if ! wait_for_transform "${ODOM_PARENT_FRAME}" "${ODOM_CHILD_FRAME}" \
    "${WAIT_TIMEOUT_SEC}" "${odom_flatten_pid}"; then
    set_component_state TF FAILED "missing=${ODOM_PARENT_FRAME}->${ODOM_CHILD_FRAME}"
    exit 1
  fi
  if ! wait_for_transform "${BASE_FRAME}" "${LIDAR_FRAME}" \
    "${WAIT_TIMEOUT_SEC}" "${static_tf_pid}"; then
    set_component_state TF FAILED "missing=${BASE_FRAME}->${LIDAR_FRAME}"
    exit 1
  fi
  set_component_state TF READY "both_required_transforms_available"

  set_component_state CANONICAL_SCAN STARTING "input=${SCAN_TOPIC} output=${RF2O_SCAN_TOPIC}"
  start_process laser_scan_canonicalizer "${CANONICALIZER_LOG}" \
    ros2 run obs_avoid laser_scan_canonicalizer --ros-args \
      -p use_sim_time:=false \
      -p input_topic:="${SCAN_TOPIC}" \
      -p output_topic:="${RF2O_SCAN_TOPIC}" \
      -p output_frame:="${LIDAR_FRAME}" \
      -p output_bins:="${RF2O_SCAN_BINS}" \
      -p output_angle_min:=-3.141592653589793 \
      -p output_angle_max:=3.141592653589793 \
      -p min_valid_input_ratio:=0.05 \
      -p diagnostics_topic:="${RF2O_SCAN_DIAGNOSTICS_TOPIC}"
  canonicalizer_pid="${LAST_STARTED_PID}"
  if validate_message_series scan "${RF2O_SCAN_TOPIC}" 5 "${RF2O_SCAN_BINS}" \
    "${canonicalizer_pid}" "${CANONICAL_SCAN_WAIT_SEC}"; then
    set_component_state CANONICAL_SCAN READY "five_monotonic_${RF2O_SCAN_BINS}_bin_scans"
  else
    canonical_status=$?
    if ! kill -0 "${RPLIDAR_PID}" 2>/dev/null; then
      set_component_state RPLIDAR FAILED "process_exited_after_initial_scan"
      set_component_state RAW_SCAN FAILED "publisher_exited"
      exit 1
    fi
    if ! kill -0 "${canonicalizer_pid}" 2>/dev/null; then
      set_component_state CANONICAL_SCAN FAILED "process_exited status=${canonical_status}"
      tail -n 30 "${CANONICALIZER_LOG}" 2>/dev/null || true
      exit 1
    fi
    set_component_state CANONICAL_SCAN DEGRADED \
      "five_scan_validation_failed status=${canonical_status}; continuing"
  fi

  set_component_state MONITOR STARTING "before_rf2o"
  start_process lidar_odom_monitor "${LIDAR_ODOM_MONITOR_LOG}" \
    ros2 run obs_avoid lidar_odom_monitor --ros-args \
      --params-file "${LIDAR_BRIDGE_PARAMS_FILE}" \
      -p use_sim_time:=false \
      -p scan_topic:="${RF2O_SCAN_TOPIC}" \
      -p raw_odom_topic:="${RF2O_RAW_ODOM_TOPIC}" \
      -p output_odom_topic:="${LIDAR_ODOM_TOPIC}" \
      -p diagnostics_topic:="${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" \
      -p expected_parent_frame:="${RF2O_ODOM_FRAME}" \
      -p expected_child_frame:="${RF2O_BASE_FRAME}" \
      -p expected_scan_bins:="${RF2O_SCAN_BINS}"
  lidar_monitor_pid="${LAST_STARTED_PID}"
  if wait_for_topic_message_alive "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" \
    "${lidar_monitor_pid}" 10 "${LIDAR_ODOM_MONITOR_LOG}"; then
    set_component_state MONITOR DEGRADED "diagnostics_active_waiting_for_rf2o"
  else
    monitor_wait_status=$?
    if ! kill -0 "${lidar_monitor_pid}" 2>/dev/null; then
      set_component_state MONITOR FAILED "process_exited status=${monitor_wait_status}"
      exit 1
    fi
    set_component_state MONITOR DEGRADED "diagnostics_not_observed status=${monitor_wait_status}"
  fi

  set_component_state RF2O STARTING "scan=${RF2O_SCAN_TOPIC}"
  start_process rf2o "${RF2O_LOG}" \
    ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
      -r __node:=rf2o_laser_odometry \
      --params-file "${RF2O_PARAMS_FILE}" \
      -p use_sim_time:=false \
      -p freq:="${RF2O_RATE_HZ}"
  rf2o_pid="${LAST_STARTED_PID}"
  if validate_message_series odom "${RF2O_RAW_ODOM_TOPIC}" 5 0 \
    "${rf2o_pid}" "${RF2O_ODOM_WAIT_SEC}"; then
    set_component_state RF2O READY "five_new_odometry_messages"
  else
    rf2o_status=$?
    if ! kill -0 "${rf2o_pid}" 2>/dev/null; then
      set_component_state RF2O FAILED "process_exited status=${rf2o_status}"
      tail -n 30 "${RF2O_LOG}" 2>/dev/null || true
      exit 1
    fi
    set_component_state RF2O DEGRADED "DEGRADED_RF2O no_five_odometry_messages status=${rf2o_status}"
    log "DEGRADED_RF2O: RF2O remains alive but has not produced five valid odometry messages"
  fi

  if ! kill -0 "${lidar_monitor_pid}" 2>/dev/null; then
    set_component_state MONITOR FAILED "process_exited_during_rf2o_startup"
    exit 1
  elif diagnostic_key_true_once "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" valid; then
    set_component_state MONITOR READY "valid=true"
  else
    set_component_state MONITOR DEGRADED "valid=false; bridge_will_remain_gated"
  fi

  if [[ "${START_LIDAR_PX4_BRIDGE}" == "1" ]]; then
    set_component_state PX4_BRIDGE STARTING "publication_gated_by_monitor"
    if ! require_vehicle_disarmed; then
      log "WARNING bridge will remain unaligned and publication-gated until the vehicle is disarmed"
    fi
    start_process lidar_odom_px4_bridge "${LIDAR_PX4_BRIDGE_LOG}" \
      ros2 run obs_avoid lidar_odom_px4_bridge --ros-args \
        --params-file "${LIDAR_BRIDGE_PARAMS_FILE}" \
        -p use_sim_time:=false \
        -p lidar_odom_topic:="${LIDAR_ODOM_TOPIC}" \
        -p monitor_diagnostics_topic:="${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" \
        -p px4_odom_topic:="${ODOM_TOPIC}" \
        -p scan_topic:="${RF2O_SCAN_TOPIC}" \
        -p output_topic:="${PX4_ODOMETRY_OUT_TOPIC}" \
        -p diagnostics_topic:="${LIDAR_PX4_DIAGNOSTICS_TOPIC}" \
        -p expected_lidar_frame:="${RF2O_ODOM_FRAME}" \
        -p health_csv_path:="${LIDAR_PX4_HEALTH_CSV}"
    lidar_bridge_pid="${LAST_STARTED_PID}"
    if wait_for_topic_message_alive "${LIDAR_PX4_DIAGNOSTICS_TOPIC}" \
      "${lidar_bridge_pid}" 10 "${LIDAR_PX4_BRIDGE_LOG}"; then
      if diagnostic_key_true_once "${LIDAR_PX4_DIAGNOSTICS_TOPIC}" alignment_complete; then
        set_component_state PX4_BRIDGE READY "aligned; monitor_publication_gate_active"
      else
        set_component_state PX4_BRIDGE DEGRADED \
          "unaligned_or_monitor_invalid; no_mavros_odometry_published"
      fi
    else
      bridge_wait_status=$?
      if ! kill -0 "${lidar_bridge_pid}" 2>/dev/null; then
        set_component_state PX4_BRIDGE FAILED "process_exited status=${bridge_wait_status}"
        exit 1
      fi
      set_component_state PX4_BRIDGE DEGRADED "diagnostics_not_observed status=${bridge_wait_status}"
    fi
    "${PX4_EKF_CHECK_SCRIPT}" >"${PX4_EKF_FUSION_SNAPSHOT}" 2>&1 ||
      log "WARNING PX4 EKF parameter snapshot is incomplete; inspect ${PX4_EKF_FUSION_SNAPSHOT}"
  else
    log "LiDAR/PX4 bridge disabled by START_LIDAR_PX4_BRIDGE=${START_LIDAR_PX4_BRIDGE}"
    set_component_state PX4_BRIDGE STOPPED "disabled"
  fi

  if [[ "${RECORD_BAG}" == "1" ]]; then
    start_process rosbag "${ROSBAG_LOG}" \
      ros2 bag record -o "${LOG_DIR}/lidar_px4_bag" \
        "${PX4_ODOMETRY_OUT_TOPIC}" "${RF2O_RAW_ODOM_TOPIC}" "${LIDAR_ODOM_TOPIC}" \
        "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" "${LIDAR_PX4_DIAGNOSTICS_TOPIC}" \
        "${RF2O_SCAN_DIAGNOSTICS_TOPIC}" "${ODOM_TOPIC}" /mavros/imu/data \
        "${SCAN_TOPIC}" "${RF2O_SCAN_TOPIC}" /tf /tf_static
  fi

  set_component_state SLAM STARTING "scan=${SCAN_TOPIC}"
  start_process slam_toolbox "${SLAM_LOG}" \
    ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:="${SLAM_PARAMS_FILE}" use_sim_time:=false
  slam_pid="${LAST_STARTED_PID}"
  if wait_for_topic_message_alive /map "${slam_pid}" "${WAIT_TIMEOUT_SEC}" "${SLAM_LOG}"; then
    set_component_state SLAM READY "map_publishing"
  else
    slam_wait_status=$?
    if ! kill -0 "${slam_pid}" 2>/dev/null; then
      set_component_state SLAM FAILED "process_exited status=${slam_wait_status}"
      exit 1
    fi
    set_component_state SLAM DEGRADED "no_map_yet status=${slam_wait_status}; continuing"
  fi

  set_component_state PLANNER STARTING "safe_console_control"
  start_process planner "${PLANNER_LOG}" \
    ros2 run obs_avoid local_planner_mode_a --ros-args \
      --params-file "${PLANNER_PARAMS_FILE}" \
      -p use_sim_time:=false \
      -r /scan_horizontal:="${SCAN_TOPIC}"
  planner_pid="${LAST_STARTED_PID}"
  sleep 2
  if kill -0 "${planner_pid}" 2>/dev/null; then
    set_component_state PLANNER READY "process_alive"
  else
    set_component_state PLANNER FAILED "process_exited_during_startup"
    tail -n 30 "${PLANNER_LOG}" 2>/dev/null || true
    exit 1
  fi

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
