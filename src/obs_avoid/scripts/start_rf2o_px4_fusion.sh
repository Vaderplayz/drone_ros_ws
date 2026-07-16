#!/usr/bin/env bash
# Independent real-drone RF2O -> MAVROS odometry launcher.
# shellcheck disable=SC1090,SC1091

set -euo pipefail

ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

RPLIDAR_SERIAL_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB0}"
RPLIDAR_BAUDRATE="${RPLIDAR_BAUDRATE:-115200}"
RPLIDAR_FRAME_ID="${RPLIDAR_FRAME_ID:-laser_frame}"
RPLIDAR_SCAN_WAIT_SEC="${RPLIDAR_SCAN_WAIT_SEC:-60}"

SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
SCAN_AUDIT_TOPIC="${SCAN_AUDIT_TOPIC:-/scan_stream_audit/diagnostics}"
CANONICAL_SCAN_TOPIC="${CANONICAL_SCAN_TOPIC:-/scan_rf2o}"
CANONICAL_DIAGNOSTICS_TOPIC="${CANONICAL_DIAGNOSTICS_TOPIC:-/scan_rf2o/diagnostics}"
RF2O_RAW_ODOM_TOPIC="${RF2O_RAW_ODOM_TOPIC:-/lidar/odom_raw}"
LIDAR_ODOM_TOPIC="${LIDAR_ODOM_TOPIC:-/lidar/odom}"
LIDAR_ODOM_DIAGNOSTICS_TOPIC="${LIDAR_ODOM_DIAGNOSTICS_TOPIC:-/lidar_odom/diagnostics}"
PX4_ODOMETRY_OUT_TOPIC="${PX4_ODOMETRY_OUT_TOPIC:-/mavros/odometry/out}"
PX4_BRIDGE_DIAGNOSTICS_TOPIC="${PX4_BRIDGE_DIAGNOSTICS_TOPIC:-/lidar_odom_px4_bridge/diagnostics}"

MAVROS_STATE_TOPIC="${MAVROS_STATE_TOPIC:-/mavros/state}"
PX4_ODOM_TOPIC="${PX4_ODOM_TOPIC:-/mavros/local_position/odom}"
PX4_IMU_TOPIC="${PX4_IMU_TOPIC:-/mavros/imu/data}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
RF2O_WAIT_SEC="${RF2O_WAIT_SEC:-60}"
PX4_BRIDGE_WAIT_SEC="${PX4_BRIDGE_WAIT_SEC:-60}"

ODOM_PARENT_FRAME="${ODOM_PARENT_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
LIDAR_FRAME="${LIDAR_FRAME:-${RPLIDAR_FRAME_ID}}"
LIDAR_X="${LIDAR_X:-0.0}"
LIDAR_Y="${LIDAR_Y:-0.0}"
LIDAR_Z="${LIDAR_Z:-0.1}"
LIDAR_ROLL="${LIDAR_ROLL:-0.0}"
LIDAR_PITCH="${LIDAR_PITCH:-0.0}"
LIDAR_YAW="${LIDAR_YAW:-0.0}"

ENABLE_SCAN_DESKEW="${ENABLE_SCAN_DESKEW:-false}"
DESKEW_FIXED_FRAME="${DESKEW_FIXED_FRAME:-${ODOM_PARENT_FRAME}}"
DESKEW_STAMP_POLICY="${DESKEW_STAMP_POLICY:-end}"
DESKEW_TIMEOUT_SEC="${DESKEW_TIMEOUT_SEC:-0.35}"
RECORD_DIAGNOSTIC_BAG="${RECORD_DIAGNOSTIC_BAG:-0}"

RF2O_PARAMS_FILE="${RF2O_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/rf2o_real_a1m8.yaml}"
ODOM_PARAMS_FILE="${ODOM_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/lidar_odom_px4_bridge.yaml}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROS_WS}/runtime_logs/rf2o_px4_${RUN_STAMP}"
MASTER_LOG="${LOG_DIR}/master.log"
RPLIDAR_LOG="${LOG_DIR}/rplidar.log"
ODOM_FLATTEN_LOG="${LOG_DIR}/odom_flatten.log"
STATIC_TF_LOG="${LOG_DIR}/static_tf.log"
SCAN_AUDIT_LOG="${LOG_DIR}/scan_audit.log"
RAW_SCAN_AUDIT_CSV="${LOG_DIR}/raw_scan_audit.csv"
CANONICALIZER_LOG="${LOG_DIR}/canonicalizer.log"
CANONICAL_HEALTH_CSV="${LOG_DIR}/canonical_scan_health.csv"
RF2O_LOG="${LOG_DIR}/rf2o.log"
ODOM_MONITOR_LOG="${LOG_DIR}/lidar_odom_monitor.log"
ODOM_HEALTH_CSV="${LOG_DIR}/lidar_odom_health.csv"
PX4_BRIDGE_LOG="${LOG_DIR}/lidar_odom_px4_bridge.log"
PX4_BRIDGE_HEALTH_CSV="${LOG_DIR}/lidar_odom_px4_bridge_health.csv"
ROSBAG_LOG="${LOG_DIR}/rosbag.log"
SNAPSHOT_FILE="${LOG_DIR}/system_snapshot.txt"

PIDS=()
NAMES=()
LAST_STARTED_PID=""
EXTERNAL_RPLIDAR=0
SHUTDOWN_REASON="normal exit"

if [[ -t 1 ]]; then
  GREEN=$'\033[1;32m'
  YELLOW=$'\033[1;33m'
  RED=$'\033[1;31m'
  RESET=$'\033[0m'
else
  GREEN=""
  YELLOW=""
  RED=""
  RESET=""
fi

timestamp() {
  date '+%Y-%m-%dT%H:%M:%S%z'
}

log() {
  printf '[%s] %s\n' "$(timestamp)" "$*"
}

log_started() {
  printf '%s[%s] STARTED %s%s\n' "${GREEN}" "$(timestamp)" "$*" "${RESET}"
}

log_warning() {
  printf '%s[%s] WARNING %s%s\n' "${YELLOW}" "$(timestamp)" "$*" "${RESET}"
}

log_error() {
  printf '%s[%s] ERROR %s%s\n' "${RED}" "$(timestamp)" "$*" "${RESET}" >&2
}

publisher_count() {
  local topic="$1" info count
  info="$(ros2 topic info "${topic}" -v 2>/dev/null || true)"
  count="$(awk '/Publisher count:/{print $3; exit}' <<<"${info}")"
  printf '%s\n' "${count:-0}"
}

require_publisher_count() {
  local topic="$1" expected="$2" actual
  actual="$(publisher_count "${topic}")"
  if [[ "${actual}" != "${expected}" ]]; then
    log_error "${topic} publisher_count=${actual}, expected=${expected}"
    ros2 topic info "${topic}" -v 2>/dev/null || true
    return 1
  fi
}

add_process() {
  local pid="$1" name="$2"
  PIDS+=("${pid}")
  NAMES+=("${name}")
  printf '%s\n' "${pid}" >"${LOG_DIR}/${name}.pid"
  log_started "${name} pid=${pid}"
}

start_process() {
  local name="$1" logfile="$2"
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

cleanup() {
  local exit_code=$? index pid
  set +e
  trap - EXIT INT TERM
  log "Shutdown reason: ${SHUTDOWN_REASON}; launcher_exit=${exit_code}"
  for ((index=${#PIDS[@]} - 1; index>=0; index--)); do
    pid="${PIDS[$index]}"
    if kill -0 -- "-${pid}" 2>/dev/null || kill -0 "${pid}" 2>/dev/null; then
      log "Stopping launcher-owned process only: ${NAMES[$index]} pid=${pid}"
      stop_process_group "${pid}"
    fi
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  if [[ "${EXTERNAL_RPLIDAR}" == "1" ]]; then
    log "The externally managed RPLIDAR process was not stopped"
  fi
  log "MAVROS, flight controller, camera, and AprilTag processes were not stopped"
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 130
}

wait_for_message() {
  local topic="$1" timeout_sec="$2" reliability="$3" pid="${4:-}"
  local logfile="${5:-}" start elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    if [[ -n "${pid}" ]] && ! kill -0 "${pid}" 2>/dev/null; then
      log_error "process pid=${pid} exited before publishing ${topic}"
      if [[ -n "${logfile}" ]]; then
        tail -n 40 "${logfile}" 2>/dev/null || true
      fi
      return 10
    fi
    if timeout 2 ros2 topic echo "${topic}" --once \
      --qos-reliability "${reliability}" >/dev/null 2>&1; then
      log_started "message stream ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "no message on ${topic} after ${timeout_sec}s"
      if [[ -n "${logfile}" ]]; then
        tail -n 40 "${logfile}" 2>/dev/null || true
      fi
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

wait_for_transform() {
  local parent="$1" child="$2" timeout_sec="$3" pid="$4"
  local start elapsed sample last_progress=-1
  start="$(date +%s)"
  while true; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      log_error "process pid=${pid} exited while waiting for ${parent} -> ${child}"
      return 10
    fi
    sample="$(timeout 2 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
    if grep -q -- '- Translation:' <<<"${sample}"; then
      log_started "TF ${parent} -> ${child}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "missing TF ${parent} -> ${child} after ${timeout_sec}s"
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for TF ${parent} -> ${child}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

wait_for_diagnostic_true() {
  local topic="$1" key="$2" pid="$3" timeout_sec="$4" logfile="$5"
  local start elapsed sample last_progress=-1
  start="$(date +%s)"
  while true; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      log_error "process pid=${pid} exited while waiting for ${topic} ${key}=true"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 10
    fi
    sample="$(timeout 3 ros2 topic echo "${topic}" --once 2>/dev/null || true)"
    if grep -A1 -F "key: ${key}" <<<"${sample}" | grep -Eq "value: ['\"]?true['\"]?"; then
      log_started "diagnostic ${topic} ${key}=true"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "diagnostic deadline exceeded: ${topic} ${key}=true"
      tail -n 40 "${logfile}" 2>/dev/null || true
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic} ${key}=true; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

validate_message_series() {
  local kind="$1" topic="$2" required="$3" expected_bins="$4" pid="$5" timeout_sec="$6"
  timeout "${timeout_sec}" python3 - "${kind}" "${topic}" "${required}" \
    "${expected_bins}" "${pid}" <<'PY'
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
else:
    from nav_msgs.msg import Odometry as Message


class Validator(Node):
    def __init__(self):
        super().__init__("rf2o_px4_startup_validator")
        self.count = 0
        self.last_stamp = -1
        self.geometry = None
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
            self.result = 10
            rclpy.shutdown()
            return
        bucket = int((time.monotonic() - self.started) // 5)
        if bucket > self.last_progress:
            self.last_progress = bucket
            print(f"waiting for {topic}: {self.count}/{required}", flush=True)

    def callback(self, msg):
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        valid = stamp > self.last_stamp and len(self.get_publishers_info_by_topic(topic)) == 1
        if kind == "scan":
            geometry = (
                len(msg.ranges), round(float(msg.angle_min), 7),
                round(float(msg.angle_max), 7), round(float(msg.angle_increment), 9),
                msg.header.frame_id)
            if self.geometry is None:
                self.geometry = geometry
            valid = (
                valid and len(msg.ranges) == expected_bins and
                all(math.isfinite(v) for v in
                    (msg.angle_min, msg.angle_max, msg.angle_increment)) and
                msg.angle_increment > 0.0 and msg.header.frame_id and
                geometry == self.geometry)
        if valid:
            self.count += 1
            self.last_stamp = stamp
        else:
            self.count = 0
            self.last_stamp = max(self.last_stamp, stamp)
        if self.count >= required:
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
}

require_existing_mavros_disarmed() {
  local start sample elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    sample="$(timeout 3 ros2 topic echo "${MAVROS_STATE_TOPIC}" --once 2>/dev/null || true)"
    if grep -Eq '^[[:space:]]*connected:[[:space:]]*true[[:space:]]*$' <<<"${sample}"; then
      if grep -Eq '^[[:space:]]*armed:[[:space:]]*true[[:space:]]*$' <<<"${sample}"; then
        log_error "vehicle is armed; RF2O/PX4 alignment requires a disarmed vehicle"
        return 1
      fi
      log_started "existing MAVROS connected; vehicle disarmed"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= WAIT_TIMEOUT_SEC )); then
      log_error "MAVROS did not report FCU connected within ${WAIT_TIMEOUT_SEC}s"
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for existing MAVROS connection; elapsed=${elapsed}s deadline=${WAIT_TIMEOUT_SEC}s"
    fi
    sleep 1
  done
}

require_no_existing_pipeline() {
  local pattern output
  local patterns=(
    '[l]aser_scan_stream_audit' '[l]aser_scan_canonicalizer'
    '[r]f2o_laser_odometry_node' '[l]idar_odom_monitor'
    '[p]x4_odom_flatten_node' '[l]idar_odom_px4_bridge'
  )
  for pattern in "${patterns[@]}"; do
    output="$(pgrep -af "${pattern}" 2>/dev/null || true)"
    if [[ -n "${output}" ]]; then
      log_error "existing process detected for ${pattern}:"
      printf '%s\n' "${output}"
      return 1
    fi
  done
  require_publisher_count "${CANONICAL_SCAN_TOPIC}" 0
  require_publisher_count "${RF2O_RAW_ODOM_TOPIC}" 0
  require_publisher_count "${LIDAR_ODOM_TOPIC}" 0
  require_publisher_count "${PX4_ODOMETRY_OUT_TOPIC}" 0
}

start_rplidar() {
  local existing_publishers pid
  existing_publishers="$(publisher_count "${SCAN_TOPIC}")"
  if [[ "${existing_publishers}" == "1" ]]; then
    wait_for_message "${SCAN_TOPIC}" 10 best_effort
    EXTERNAL_RPLIDAR=1
    log_started "reusing one externally managed RPLIDAR publisher"
    return 0
  fi
  if [[ "${existing_publishers}" != "0" ]]; then
    log_error "${SCAN_TOPIC} publisher_count=${existing_publishers}, expected 0 or 1"
    return 1
  fi
  if pgrep -af '[r]plidar_composition|[r]plidar_node' >/dev/null 2>&1; then
    log_error "RPLIDAR process exists but ${SCAN_TOPIC} has no publisher; stop that process manually"
    pgrep -af '[r]plidar_composition|[r]plidar_node' || true
    return 1
  fi
  start_process rplidar "${RPLIDAR_LOG}" \
    ros2 run rplidar_ros rplidar_composition --ros-args \
      -p channel_type:=serial -p serial_port:="${RPLIDAR_SERIAL_PORT}" \
      -p serial_baudrate:="${RPLIDAR_BAUDRATE}" -p frame_id:="${RPLIDAR_FRAME_ID}" \
      -p inverted:=false -p angle_compensate:=true -p topic_name:="${SCAN_TOPIC#/}" \
      -p use_sim_time:="${USE_SIM_TIME}"
  pid="${LAST_STARTED_PID}"
  wait_for_message "${SCAN_TOPIC}" "${RPLIDAR_SCAN_WAIT_SEC}" best_effort "${pid}" "${RPLIDAR_LOG}"
  require_publisher_count "${SCAN_TOPIC}" 1
}

start_tf_support() {
  start_process odom_flatten "${ODOM_FLATTEN_LOG}" \
    ros2 run odom_flatten px4_odom_flatten_node --ros-args \
      -p use_sim_time:="${USE_SIM_TIME}" -p odom_topic:="${PX4_ODOM_TOPIC}" \
      -p parent_frame:="${ODOM_PARENT_FRAME}" -p child_frame:="${BASE_FRAME}"
  local odom_pid="${LAST_STARTED_PID}"

  start_process static_lidar_tf "${STATIC_TF_LOG}" \
    ros2 run tf2_ros static_transform_publisher \
      --x "${LIDAR_X}" --y "${LIDAR_Y}" --z "${LIDAR_Z}" \
      --roll "${LIDAR_ROLL}" --pitch "${LIDAR_PITCH}" --yaw "${LIDAR_YAW}" \
      --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR_FRAME}" \
      --ros-args -p use_sim_time:="${USE_SIM_TIME}"
  local static_pid="${LAST_STARTED_PID}"

  wait_for_transform "${ODOM_PARENT_FRAME}" "${BASE_FRAME}" "${WAIT_TIMEOUT_SEC}" "${odom_pid}"
  wait_for_transform "${BASE_FRAME}" "${LIDAR_FRAME}" "${WAIT_TIMEOUT_SEC}" "${static_pid}"
}

start_scan_conditioning() {
  start_process scan_audit "${SCAN_AUDIT_LOG}" \
    ros2 run obs_avoid laser_scan_stream_audit --ros-args \
      -p input_topic:="${SCAN_TOPIC}" -p diagnostics_topic:="${SCAN_AUDIT_TOPIC}" \
      -p csv_path:="${RAW_SCAN_AUDIT_CSV}" -p classification_window_messages:=20 \
      -p full_revolution_min_span_rad:=5.8 -p use_sim_time:="${USE_SIM_TIME}"
  local audit_pid="${LAST_STARTED_PID}"
  wait_for_diagnostic_true "${SCAN_AUDIT_TOPIC}" classification_ready \
    "${audit_pid}" 30 "${SCAN_AUDIT_LOG}"

  start_process canonicalizer "${CANONICALIZER_LOG}" \
    ros2 run obs_avoid laser_scan_canonicalizer --ros-args \
      -p input_topic:="${SCAN_TOPIC}" -p output_topic:="${CANONICAL_SCAN_TOPIC}" \
      -p output_frame:="${LIDAR_FRAME}" -p processing_mode:=auto -p output_bins:=720 \
      -p output_angle_min:=-3.141592653589793 -p output_angle_max:=3.141592653589793 \
      -p classification_window_messages:=20 -p full_revolution_min_span_rad:=5.8 \
      -p minimum_angular_coverage_ratio:=0.70 -p minimum_finite_return_ratio:=0.05 \
      -p maximum_revolution_duration_sec:=0.50 -p maximum_segment_gap_sec:=0.20 \
      -p maximum_input_messages_per_revolution:=20 -p publish_rate_limit_hz:=12.0 \
      -p diagnostics_rate_hz:=1.0 -p diagnostics_topic:="${CANONICAL_DIAGNOSTICS_TOPIC}" \
      -p enable_deskew:="${ENABLE_SCAN_DESKEW}" -p deskew_fixed_frame:="${DESKEW_FIXED_FRAME}" \
      -p deskew_stamp_policy:="${DESKEW_STAMP_POLICY}" \
      -p deskew_timeout_sec:="${DESKEW_TIMEOUT_SEC}" -p use_sim_time:="${USE_SIM_TIME}" \
      -p health_csv_path:="${CANONICAL_HEALTH_CSV}"
  local canonicalizer_pid="${LAST_STARTED_PID}"
  validate_message_series scan "${CANONICAL_SCAN_TOPIC}" 5 720 \
    "${canonicalizer_pid}" 60
  require_publisher_count "${CANONICAL_SCAN_TOPIC}" 1
  wait_for_diagnostic_true "${CANONICAL_DIAGNOSTICS_TOPIC}" timestamp_monotonic \
    "${canonicalizer_pid}" 15 "${CANONICALIZER_LOG}"
  log_started "five coherent canonical scans"
}

start_rf2o_and_monitor() {
  start_process lidar_odom_monitor "${ODOM_MONITOR_LOG}" \
    ros2 run obs_avoid lidar_odom_monitor --ros-args --params-file "${ODOM_PARAMS_FILE}" \
      -p health_csv_path:="${ODOM_HEALTH_CSV}" -p use_sim_time:="${USE_SIM_TIME}"
  local monitor_pid="${LAST_STARTED_PID}"
  sleep 1
  if ! kill -0 "${monitor_pid}" 2>/dev/null; then
    log_error "LiDAR odometry monitor exited during startup"
    tail -n 40 "${ODOM_MONITOR_LOG}" 2>/dev/null || true
    return 1
  fi

  start_process rf2o "${RF2O_LOG}" \
    ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
      -r __node:=rf2o_laser_odometry --params-file "${RF2O_PARAMS_FILE}" \
      -p use_sim_time:="${USE_SIM_TIME}"
  local rf2o_pid="${LAST_STARTED_PID}"
  validate_message_series odom "${RF2O_RAW_ODOM_TOPIC}" 5 0 "${rf2o_pid}" "${RF2O_WAIT_SEC}"
  require_publisher_count "${RF2O_RAW_ODOM_TOPIC}" 1
  log_started "RF2O raw odometry ${RF2O_RAW_ODOM_TOPIC}"

  validate_message_series odom "${LIDAR_ODOM_TOPIC}" 5 0 "${monitor_pid}" "${RF2O_WAIT_SEC}"
  require_publisher_count "${LIDAR_ODOM_TOPIC}" 1
  log_started "health-gated LiDAR odometry ${LIDAR_ODOM_TOPIC}"
}

start_px4_bridge() {
  require_existing_mavros_disarmed
  start_process lidar_odom_px4_bridge "${PX4_BRIDGE_LOG}" \
    ros2 run obs_avoid lidar_odom_px4_bridge --ros-args --params-file "${ODOM_PARAMS_FILE}" \
      -p health_csv_path:="${PX4_BRIDGE_HEALTH_CSV}" -p use_sim_time:="${USE_SIM_TIME}"
  local bridge_pid="${LAST_STARTED_PID}"
  wait_for_message "${PX4_ODOMETRY_OUT_TOPIC}" "${PX4_BRIDGE_WAIT_SEC}" reliable \
    "${bridge_pid}" "${PX4_BRIDGE_LOG}"
  require_publisher_count "${PX4_ODOMETRY_OUT_TOPIC}" 1
  log_started "RF2O feed to PX4 via ${PX4_ODOMETRY_OUT_TOPIC}"
}

start_diagnostic_bag() {
  if [[ "${RECORD_DIAGNOSTIC_BAG}" != "1" ]]; then
    return 0
  fi
  start_process diagnostic_bag "${ROSBAG_LOG}" \
    ros2 bag record -o "${LOG_DIR}/diagnostic_bag" \
      "${SCAN_TOPIC}" "${CANONICAL_SCAN_TOPIC}" "${SCAN_AUDIT_TOPIC}" \
      "${CANONICAL_DIAGNOSTICS_TOPIC}" "${RF2O_RAW_ODOM_TOPIC}" \
      "${LIDAR_ODOM_TOPIC}" "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" \
      "${PX4_ODOMETRY_OUT_TOPIC}" "${PX4_BRIDGE_DIAGNOSTICS_TOPIC}" \
      "${PX4_ODOM_TOPIC}" "${PX4_IMU_TOPIC}" /tf /tf_static
}

write_snapshot() {
  {
    printf 'timestamp=%s\nworkspace=%s\nuse_sim_time=%s\n' \
      "$(timestamp)" "${ROS_WS}" "${USE_SIM_TIME}"
    printf 'rplidar_port=%s\ndeskew_enabled=%s\ndeskew_stamp_policy=%s\n' \
      "${RPLIDAR_SERIAL_PORT}" "${ENABLE_SCAN_DESKEW}" "${DESKEW_STAMP_POLICY}"
    for topic in "${SCAN_TOPIC}" "${CANONICAL_SCAN_TOPIC}" "${RF2O_RAW_ODOM_TOPIC}" \
      "${LIDAR_ODOM_TOPIC}" "${PX4_ODOMETRY_OUT_TOPIC}"; do
      printf '\n-- %s --\n' "${topic}"
      ros2 topic info "${topic}" -v || true
    done
    printf '\n-- LiDAR odometry diagnostics --\n'
    timeout 5 ros2 topic echo "${LIDAR_ODOM_DIAGNOSTICS_TOPIC}" --once || true
    printf '\n-- PX4 bridge diagnostics --\n'
    timeout 5 ros2 topic echo "${PX4_BRIDGE_DIAGNOSTICS_TOPIC}" --once || true
  } >"${SNAPSHOT_FILE}" 2>&1
}

ready_banner() {
  printf '\n%s' "${GREEN}"
  printf '######################################################################\n'
  printf '#                                                                    #\n'
  printf '#                         ALL SYSTEM READY                           #\n'
  printf '#                                                                    #\n'
  printf '#             RF2O odometry is feeding PX4 through MAVROS            #\n'
  printf '#                                                                    #\n'
  printf '######################################################################\n'
  printf '%s\n' "${RESET}"
}

main() {
  mkdir -p "${LOG_DIR}"
  touch "${MASTER_LOG}"
  exec > >(tee -a "${MASTER_LOG}") 2>&1
  trap cleanup EXIT
  trap 'on_signal INT' INT
  trap 'on_signal TERM' TERM

  if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "${ROS_SETUP}" ]]; then
    log_error "ROS setup is missing"
    exit 1
  fi
  if [[ ! -f "${RF2O_PARAMS_FILE}" || ! -f "${ODOM_PARAMS_FILE}" ]]; then
    log_error "RF2O or odometry configuration is missing"
    exit 1
  fi
  if [[ ! -e "${RPLIDAR_SERIAL_PORT}" || ! -r "${RPLIDAR_SERIAL_PORT}" ||
    ! -w "${RPLIDAR_SERIAL_PORT}" ]]; then
    log_error "${RPLIDAR_SERIAL_PORT} is absent or not readable/writable by $(id -un)"
    exit 1
  fi

  set +u
  source /opt/ros/jazzy/setup.bash
  source "${ROS_SETUP}"
  set -u

  for command in ros2 timeout pgrep setsid python3 awk; do
    if ! command -v "${command}" >/dev/null 2>&1; then
      log_error "missing command: ${command}"
      exit 1
    fi
  done

  log "Independent RF2O/PX4 odometry startup"
  log "Workspace=${ROS_WS}; LiDAR=${RPLIDAR_SERIAL_PORT}; use_sim_time=${USE_SIM_TIME}"
  log "This launcher will not manage MAVROS, the flight controller, camera, or AprilTag nodes"

  require_existing_mavros_disarmed
  require_no_existing_pipeline
  start_diagnostic_bag
  start_rplidar
  start_tf_support
  start_scan_conditioning
  start_rf2o_and_monitor
  start_px4_bridge
  write_snapshot
  ready_banner

  log "ROS/MAVROS feed is ready; verify EKF2 external-vision aid flags before flight"
  log "No PX4 parameter, arm, mode, or movement command was sent"
  log "Runtime logs: ${LOG_DIR}"
  while true; do
    local index
    for index in "${!PIDS[@]}"; do
      if ! kill -0 "${PIDS[$index]}" 2>/dev/null; then
        SHUTDOWN_REASON="required process exited: ${NAMES[$index]} pid=${PIDS[$index]}"
        log_error "${SHUTDOWN_REASON}"
        return 1
      fi
    done
    sleep 2
  done
}

main "$@"
