#!/usr/bin/env bash
# Observer-only 2D mapping. Requires the independent RF2O/PX4 pipeline to be running.
# shellcheck disable=SC1090,SC1091

set -euo pipefail

ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar.yaml}"
ENABLE_SUBMAP_SLAM="${ENABLE_SUBMAP_SLAM:-1}"
SUBMAP_PARAMS_FILE="${SUBMAP_PARAMS_FILE:-${ROS_WS}/src/submap_slam_2d/config/real_rf2o_submap.yaml}"
SOURCE_SCAN_TOPIC="${SOURCE_SCAN_TOPIC:-/scan_rf2o}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_slam}"
SCAN_DIAGNOSTICS_TOPIC="${SCAN_DIAGNOSTICS_TOPIC:-/scan_slam/diagnostics}"
SUBMAP_MAP_TOPIC="${SUBMAP_MAP_TOPIC:-/submap_slam/map}"
SUBMAP_DIAGNOSTICS_TOPIC="${SUBMAP_DIAGNOSTICS_TOPIC:-/submap_slam/diagnostics}"
ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/odom}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
MAP_FRAME="${MAP_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
LIDAR_FRAME="${LIDAR_FRAME:-laser_frame}"
INPUT_WAIT_SEC="${INPUT_WAIT_SEC:-60}"
MAP_WAIT_SEC="${MAP_WAIT_SEC:-120}"
PROCESS_STOP_TIMEOUT_SEC="${PROCESS_STOP_TIMEOUT_SEC:-5}"
MAX_INPUT_AGE_SEC="${MAX_INPUT_AGE_SEC:-10}"
MAX_INPUT_STAMP_DELTA_SEC="${MAX_INPUT_STAMP_DELTA_SEC:-10}"
DESKEW_STAMP_POLICY="${DESKEW_STAMP_POLICY:-end}"
DESKEW_TIMEOUT_SEC="${DESKEW_TIMEOUT_SEC:-0.35}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROS_WS}/runtime_logs/mapping_2d_${RUN_STAMP}"
MASTER_LOG="${LOG_DIR}/master.log"
DESKEW_LOG="${LOG_DIR}/scan_deskew.log"
DESKEW_HEALTH_CSV="${LOG_DIR}/scan_deskew_health.csv"
SLAM_LOG="${LOG_DIR}/slam_toolbox.log"
SUBMAP_SLAM_LOG="${LOG_DIR}/submap_slam_2d.log"
SNAPSHOT_FILE="${LOG_DIR}/system_snapshot.txt"
MAP_SAVE_DIR="${MAP_SAVE_DIR:-${ROS_WS}/maps}"
AUTO_SAVE_2D_MAP_ON_EXIT="${AUTO_SAVE_2D_MAP_ON_EXIT:-1}"
MAP_SAVE_PREFIX="${MAP_SAVE_PREFIX:-indoor_map}"
MAP_SAVE_TIMEOUT_SEC="${MAP_SAVE_TIMEOUT_SEC:-20}"

STATE_ROOT="${XDG_RUNTIME_DIR:-/tmp}/mapping_2d_only_$(id -u)"
LOCK_FILE="${STATE_ROOT}/launcher.lock"
OWNER_FILE="${STATE_ROOT}/launcher.pid"
PROCESS_FILE="${STATE_ROOT}/slam_process.tsv"
BOOT_ID="$(< /proc/sys/kernel/random/boot_id)"

PIDS=()
NAMES=()
DESKEW_PID=""
SLAM_PID=""
SUBMAP_SLAM_PID=""
LAST_STARTED_PID=""
LOCK_ACQUIRED=0
SHUTDOWN_REASON="normal exit"
CONTROL_TOPICS=(
  /cmd_vel
  /planner_cmd_vel
  /mavros/setpoint_velocity/cmd_vel
  /mavros/setpoint_raw/local
)
CONTROL_COUNTS=()

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

validate_launcher_settings() {
  local value topic frame launch_arguments
  case "${USE_SIM_TIME}" in
    true|false) ;;
    *)
      log_error "USE_SIM_TIME must be exactly true or false, got '${USE_SIM_TIME}'"
      return 1 ;;
  esac
  case "${ENABLE_SUBMAP_SLAM}" in
    0|1) ;;
    *)
      log_error "ENABLE_SUBMAP_SLAM must be 0 or 1, got '${ENABLE_SUBMAP_SLAM}'"
      return 1 ;;
  esac
  for value in "${INPUT_WAIT_SEC}" "${MAP_WAIT_SEC}" "${PROCESS_STOP_TIMEOUT_SEC}" \
    "${MAX_INPUT_AGE_SEC}" "${MAX_INPUT_STAMP_DELTA_SEC}"; do
    if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
      log_error "timeout and timestamp limits must be positive integer seconds, got '${value}'"
      return 1
    fi
  done
  for topic in "${SOURCE_SCAN_TOPIC}" "${SCAN_TOPIC}" "${SCAN_DIAGNOSTICS_TOPIC}" \
    "${ODOM_TOPIC}" "${MAP_TOPIC}" "${SUBMAP_MAP_TOPIC}" "${SUBMAP_DIAGNOSTICS_TOPIC}"; do
    if [[ ! "${topic}" =~ ^/[A-Za-z0-9_/]+$ || "${topic}" == *//* ||
      "${topic}" == */ ]]; then
      log_error "invalid absolute ROS topic name '${topic}'"
      return 1
    fi
  done
  if [[ "${SOURCE_SCAN_TOPIC}" == "${SCAN_TOPIC}" ]]; then
    log_error "SOURCE_SCAN_TOPIC and SCAN_TOPIC must differ so deskew cannot feed itself"
    return 1
  fi
  case "${DESKEW_STAMP_POLICY}" in
    start|end) ;;
    *)
      log_error "DESKEW_STAMP_POLICY must be start or end, got '${DESKEW_STAMP_POLICY}'"
      return 1 ;;
  esac
  if [[ ! "${DESKEW_TIMEOUT_SEC}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    log_error "DESKEW_TIMEOUT_SEC must be a non-negative number, got '${DESKEW_TIMEOUT_SEC}'"
    return 1
  fi
  for frame in "${MAP_FRAME}" "${ODOM_FRAME}" "${BASE_FRAME}" "${LIDAR_FRAME}"; do
    if [[ ! "${frame}" =~ ^[A-Za-z][A-Za-z0-9_/]*$ || "${frame}" == *//* ||
      "${frame}" == */ ]]; then
      log_error "invalid TF frame name '${frame}'"
      return 1
    fi
  done

  if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
    log_error "slam_toolbox package is unavailable"
    return 1
  fi
  if ! ros2 pkg executables obs_avoid 2>/dev/null | \
    grep -q '^obs_avoid laser_scan_canonicalizer$'; then
    log_error "obs_avoid laser_scan_canonicalizer executable is unavailable"
    return 1
  fi
  if ! ros2 pkg executables slam_toolbox 2>/dev/null | \
    grep -q '^slam_toolbox async_slam_toolbox_node$'; then
    log_error "slam_toolbox async executable is unavailable"
    return 1
  fi
  launch_arguments="$(timeout 10 ros2 launch slam_toolbox online_async_launch.py \
    --show-args 2>/dev/null || true)"
  if [[ "${launch_arguments}" != *"'slam_params_file'"* ||
    "${launch_arguments}" != *"'use_sim_time'"* ]]; then
    log_error "installed online_async_launch.py lacks required launch arguments"
    return 1
  fi
  if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
    log_warning "nav2_map_server is unavailable; live mapping works but map_saver_cli will not"
  fi
  if [[ "${ENABLE_SUBMAP_SLAM}" == "1" ]]; then
    if [[ ! -f "${SUBMAP_PARAMS_FILE}" ]]; then
      log_error "submap SLAM configuration is missing: ${SUBMAP_PARAMS_FILE}"
      return 1
    fi
    if ! ros2 pkg executables submap_slam_2d 2>/dev/null | \
      grep -q '^submap_slam_2d submap_slam_2d_node$'; then
      log_error "submap_slam_2d is enabled but its node is unavailable; rebuild the workspace"
      return 1
    fi
  fi
  log_started "launcher inputs and installed ROS interfaces validated"
}

process_group_alive() {
  local pid="$1"
  kill -0 -- "-${pid}" 2>/dev/null || kill -0 "${pid}" 2>/dev/null
}

signal_process_group() {
  local pid="$1" signal="$2"
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  elif kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

start_process() {
  local name="$1" logfile="$2"
  shift 2
  setsid "$@" 9>&- >"${logfile}" 2>&1 &
  LAST_STARTED_PID="$!"
  PIDS+=("${LAST_STARTED_PID}")
  NAMES+=("${name}")
  printf '%s\t%s\t%s\n' "${LAST_STARTED_PID}" "${name}" "${BOOT_ID}" >>"${PROCESS_FILE}"
  log_started "${name} pgid=${LAST_STARTED_PID}"
}

stop_owned_groups() {
  local deadline any_alive index pid
  for ((index=${#PIDS[@]} - 1; index>=0; index--)); do
    pid="${PIDS[$index]}"
    if process_group_alive "${pid}"; then
      log "Stopping mapping-owned process group: ${NAMES[$index]} pgid=${pid}"
      signal_process_group "${pid}" TERM
    fi
  done

  deadline=$(( $(date +%s) + PROCESS_STOP_TIMEOUT_SEC ))
  while (( $(date +%s) < deadline )); do
    any_alive=0
    for pid in "${PIDS[@]}"; do
      if process_group_alive "${pid}"; then
        any_alive=1
        break
      fi
    done
    (( any_alive == 0 )) && break
    sleep 0.2
  done

  for ((index=${#PIDS[@]} - 1; index>=0; index--)); do
    pid="${PIDS[$index]}"
    if process_group_alive "${pid}"; then
      log_warning "forcing stopped process group: ${NAMES[$index]} pgid=${pid}"
      signal_process_group "${pid}" KILL
    fi
  done
}

save_2d_map_on_exit() {
  local save_prefix pgm_path png_path yaml_path
  if [[ "${AUTO_SAVE_2D_MAP_ON_EXIT}" != "1" ]]; then
    log "2D map autosave disabled by AUTO_SAVE_2D_MAP_ON_EXIT=${AUTO_SAVE_2D_MAP_ON_EXIT}"
    return 0
  fi
  if [[ -z "${SLAM_PID}" ]] || ! process_group_alive "${SLAM_PID}"; then
    log_warning "2D map autosave skipped because slam_toolbox is not running"
    return 0
  fi
  if ! timeout 5 ros2 topic echo "${MAP_TOPIC}" --once \
    --qos-reliability reliable --qos-durability transient_local >/dev/null 2>&1; then
    log_warning "2D map autosave skipped because ${MAP_TOPIC} has no readable map"
    return 0
  fi
  if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
    log_warning "2D map autosave skipped because nav2_map_server is unavailable"
    return 0
  fi

  mkdir -p "${MAP_SAVE_DIR}"
  save_prefix="${MAP_SAVE_DIR}/${MAP_SAVE_PREFIX}_${RUN_STAMP}"
  log "Autosaving 2D map to ${save_prefix}.yaml/.pgm"
  if ! timeout "${MAP_SAVE_TIMEOUT_SEC}" ros2 run nav2_map_server map_saver_cli \
    -f "${save_prefix}" >/dev/null 2>&1; then
    log_warning "2D map autosave failed; map_saver_cli did not complete within ${MAP_SAVE_TIMEOUT_SEC}s"
    return 0
  fi

  pgm_path="${save_prefix}.pgm"
  png_path="${save_prefix}.png"
  yaml_path="${save_prefix}.yaml"
  if [[ ! -s "${pgm_path}" || ! -s "${yaml_path}" ]]; then
    log_warning "2D map autosave did not produce expected files: ${yaml_path}, ${pgm_path}"
    return 0
  fi

  if timeout 10 python3 - "${pgm_path}" "${png_path}" <<'PY'
import re
import struct
import sys
import zlib

pgm_path, png_path = sys.argv[1:3]
data = open(pgm_path, "rb").read()
index = 0

def token():
    global index
    while index < len(data):
        if data[index:index + 1] == b"#":
            end = data.find(b"\n", index)
            index = len(data) if end < 0 else end + 1
        elif data[index:index + 1].isspace():
            index += 1
        else:
            break
    match = re.match(rb"\S+", data[index:])
    if not match:
        raise RuntimeError("bad PGM header")
    value = match.group(0)
    index += len(value)
    return value

magic = token()
width = int(token())
height = int(token())
max_value = int(token())
while index < len(data) and data[index:index + 1].isspace():
    index += 1
if magic == b"P5":
    values = data[index:index + width * height]
elif magic == b"P2":
    values = [int(v) for v in re.findall(rb"\d+", data[index:])[:width * height]]
else:
    raise RuntimeError(f"unsupported PGM magic {magic!r}")
if max_value != 255:
    pixels = bytes(round(v * 255 / max_value) for v in values)
else:
    pixels = bytes(values)
if len(pixels) != width * height:
    raise RuntimeError("PGM pixel count mismatch")

def chunk(kind, payload):
    body = kind + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

raw = b"".join(b"\x00" + pixels[row * width:(row + 1) * width] for row in range(height))
png = (
    b"\x89PNG\r\n\x1a\n" +
    chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)) +
    chunk(b"IDAT", zlib.compress(raw, 9)) +
    chunk(b"IEND", b"")
)
open(png_path, "wb").write(png)
PY
  then
    log "2D map autosaved: ${yaml_path}, ${pgm_path}, ${png_path}"
  else
    log_warning "2D map PGM saved, but PNG conversion failed: ${pgm_path}"
  fi
}

cleanup() {
  local exit_code=$?
  set +e
  trap - EXIT INT TERM
  log "Shutdown reason: ${SHUTDOWN_REASON}; launcher_exit=${exit_code}"
  save_2d_map_on_exit
  stop_owned_groups
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  if [[ "${LOCK_ACQUIRED}" == "1" ]]; then
    : >"${PROCESS_FILE}"
    : >"${OWNER_FILE}"
  fi
  log "RF2O/PX4 fusion, MAVROS, LiDAR, camera, AprilTag, and flight control were not stopped"
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 130
}

recover_previous_mapping() {
  local recorded_pid recorded_name recorded_boot deadline any_alive pid
  local recovered_pids=()
  [[ -s "${PROCESS_FILE}" ]] || return 0

  while IFS=$'\t' read -r recorded_pid recorded_name recorded_boot; do
    [[ "${recorded_pid:-}" =~ ^[0-9]+$ ]] || continue
    if [[ -z "${recorded_boot:-}" ]]; then
      recorded_boot="${recorded_name:-}"
      recorded_name="legacy_mapping_process"
    fi
    [[ "${recorded_boot}" == "${BOOT_ID}" ]] || continue
    if process_group_alive "${recorded_pid}"; then
      recovered_pids+=("${recorded_pid}")
    fi
  done <"${PROCESS_FILE}"

  (( ${#recovered_pids[@]} > 0 )) || return 0
  log_warning "recovering ${#recovered_pids[@]} mapping-owned process group(s) from an interrupted run"
  for pid in "${recovered_pids[@]}"; do
    signal_process_group "${pid}" TERM
  done
  deadline=$(( $(date +%s) + PROCESS_STOP_TIMEOUT_SEC ))
  while (( $(date +%s) < deadline )); do
    any_alive=0
    for pid in "${recovered_pids[@]}"; do
      if process_group_alive "${pid}"; then
        any_alive=1
        break
      fi
    done
    (( any_alive == 0 )) && break
    sleep 0.2
  done
  for pid in "${recovered_pids[@]}"; do
    if process_group_alive "${pid}"; then
      signal_process_group "${pid}" KILL
    fi
  done
  log_started "previous mapping-owned process groups cleared"
}

acquire_mapping_lock() {
  mkdir -p "${STATE_ROOT}"
  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    log_error "another 2D mapping launcher is already running"
    [[ -s "${OWNER_FILE}" ]] && log_error "launcher pid=$(<"${OWNER_FILE}")"
    return 1
  fi
  LOCK_ACQUIRED=1
  printf '%s\n' "$$" >"${OWNER_FILE}"
  recover_previous_mapping
  : >"${PROCESS_FILE}"
}

wait_for_message() {
  local topic="$1" timeout_sec="$2" reliability="$3" pid="${4:-}" logfile="${5:-}"
  local start elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    if [[ -n "${pid}" ]] && ! kill -0 "${pid}" 2>/dev/null; then
      log_error "process pid=${pid} exited before publishing ${topic}"
      if [[ -n "${logfile}" ]]; then
        tail -n 40 "${logfile}" 2>/dev/null || true
      fi
      return 1
    fi
    if timeout 3 ros2 topic echo "${topic}" --once \
      --qos-reliability "${reliability}" >/dev/null 2>&1; then
      log_started "live input ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "no message on required input ${topic} after ${timeout_sec}s"
      if [[ -n "${logfile}" ]]; then
        tail -n 40 "${logfile}" 2>/dev/null || true
      fi
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic}; elapsed=${elapsed}s deadline=${timeout_sec}s"
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
      return 1
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
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic} ${key}=true; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

wait_for_transform() {
  local parent="$1" child="$2" timeout_sec="$3"
  local start elapsed output last_progress=-1
  start="$(date +%s)"
  while true; do
    output="$(timeout 3 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
    if grep -q -- '- Translation:' <<<"${output}"; then
      log_started "TF ${parent} -> ${child}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "missing required TF ${parent} -> ${child} after ${timeout_sec}s"
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for TF ${parent} -> ${child}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

capture_control_baseline() {
  local topic
  CONTROL_COUNTS=()
  for topic in "${CONTROL_TOPICS[@]}"; do
    CONTROL_COUNTS+=("$(publisher_count "${topic}")")
  done
}

verify_no_control_publishers_added() {
  local index topic current changed=0
  for index in "${!CONTROL_TOPICS[@]}"; do
    topic="${CONTROL_TOPICS[$index]}"
    current="$(publisher_count "${topic}")"
    if [[ "${current}" != "${CONTROL_COUNTS[$index]}" ]]; then
      changed=1
      log_warning "external publisher count changed on ${topic}: ${CONTROL_COUNTS[$index]} -> ${current}"
    fi
  done
  if (( changed == 0 )); then
    log_started "no flight-control or planner publishers added"
  else
    log_warning "mapping launcher owns only mapping observers; inspect independently changed control publishers"
  fi
}

require_clean_mapping_namespace() {
  local nodes existing_map_tf
  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -qx '/slam_toolbox' <<<"${nodes}"; then
    log_error "an unowned /slam_toolbox node is already running"
    return 1
  fi
  if grep -qx '/mapping_scan_deskew' <<<"${nodes}"; then
    log_error "an unowned /mapping_scan_deskew node is already running"
    return 1
  fi
  if [[ "${ENABLE_SUBMAP_SLAM}" == "1" ]] && grep -qx '/submap_slam_2d' <<<"${nodes}"; then
    log_error "an unowned /submap_slam_2d node is already running"
    return 1
  fi
  if [[ "$(publisher_count "${SCAN_TOPIC}")" != "0" ]]; then
    log_error "${SCAN_TOPIC} already has a publisher; refusing a second deskew source"
    ros2 topic info "${SCAN_TOPIC}" -v 2>/dev/null || true
    return 1
  fi
  if [[ "$(publisher_count "${MAP_TOPIC}")" != "0" ]]; then
    log_error "${MAP_TOPIC} already has a publisher; refusing a second map source"
    ros2 topic info "${MAP_TOPIC}" -v 2>/dev/null || true
    return 1
  fi
  if [[ "${ENABLE_SUBMAP_SLAM}" == "1" ]] && \
    [[ "$(publisher_count "${SUBMAP_MAP_TOPIC}")" != "0" ]]; then
    log_error "${SUBMAP_MAP_TOPIC} already has a publisher; refusing a second submap mapper"
    ros2 topic info "${SUBMAP_MAP_TOPIC}" -v 2>/dev/null || true
    return 1
  fi
  existing_map_tf="$(timeout 3 ros2 run tf2_ros tf2_echo \
    "${MAP_FRAME}" "${ODOM_FRAME}" 2>&1 || true)"
  if grep -q -- '- Translation:' <<<"${existing_map_tf}"; then
    log_error "TF ${MAP_FRAME} -> ${ODOM_FRAME} already exists; refusing a second mapping authority"
    return 1
  fi
}

topic_stamp_seconds() {
  local topic="$1" reliability="$2" sample stamp
  sample="$(timeout 5 ros2 topic echo "${topic}" --once --field header \
    --qos-reliability "${reliability}" 2>/dev/null || true)"
  stamp="$(awk '/^[[:space:]]*sec:/{print $2; exit}' <<<"${sample}")"
  [[ "${stamp}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${stamp}"
}

verify_live_timestamp_domain() {
  local scan_topic="$1" scan_stamp odom_stamp delta wall_time scan_age odom_age
  scan_stamp="$(topic_stamp_seconds "${scan_topic}" best_effort)" || {
    log_error "could not read a timestamp from ${scan_topic}"
    return 1
  }
  odom_stamp="$(topic_stamp_seconds "${ODOM_TOPIC}" best_effort)" || {
    log_error "could not read a timestamp from ${ODOM_TOPIC}"
    return 1
  }

  delta=$((scan_stamp - odom_stamp))
  (( delta < 0 )) && delta=$((-delta))
  if (( delta > MAX_INPUT_STAMP_DELTA_SEC )); then
    log_error "input clock mismatch: ${scan_topic}=${scan_stamp}, ${ODOM_TOPIC}=${odom_stamp}, delta=${delta}s"
    return 1
  fi

  if [[ "${USE_SIM_TIME}" == "false" ]]; then
    wall_time="$(date +%s)"
    scan_age=$((wall_time - scan_stamp))
    odom_age=$((wall_time - odom_stamp))
    (( scan_age < 0 )) && scan_age=$((-scan_age))
    (( odom_age < 0 )) && odom_age=$((-odom_age))
    if (( scan_age > MAX_INPUT_AGE_SEC || odom_age > MAX_INPUT_AGE_SEC )); then
      log_error "stale/non-wall-clock inputs: scan_age=${scan_age}s odom_age=${odom_age}s"
      return 1
    fi
  fi
  log_started "live scan and odometry timestamps share one clock domain"
}

validate_slam_node() {
  local node_info
  node_info="$(timeout 5 ros2 node info /slam_toolbox 2>/dev/null || true)"
  if [[ -z "${node_info}" ]]; then
    log_warning "slam_toolbox graph introspection unavailable; /map and full TF chain already verified"
    return 0
  fi
  if ! grep -qF "${SCAN_TOPIC}:" <<<"${node_info}"; then
    log_error "slam_toolbox is not subscribed to ${SCAN_TOPIC}"
    printf '%s\n' "${node_info}"
    return 1
  fi
  for topic in "${CONTROL_TOPICS[@]}"; do
    if grep -qF "${topic}:" <<<"${node_info}"; then
      log_error "slam_toolbox unexpectedly exposes control topic ${topic}"
      return 1
    fi
  done
  log_started "slam_toolbox scan subscription verified"
}

start_deskew() {
  start_process mapping_scan_deskew "${DESKEW_LOG}" \
    ros2 run obs_avoid laser_scan_canonicalizer --ros-args \
      -r __node:=mapping_scan_deskew \
      -p input_topic:="${SOURCE_SCAN_TOPIC}" -p output_topic:="${SCAN_TOPIC}" \
      -p diagnostics_topic:="${SCAN_DIAGNOSTICS_TOPIC}" -p output_frame:="${LIDAR_FRAME}" \
      -p processing_mode:=per_message_full_scan -p output_bins:=720 \
      -p output_angle_min:=-3.141592653589793 -p output_angle_max:=3.141592653589793 \
      -p classification_window_messages:=20 -p full_revolution_min_span_rad:=5.8 \
      -p minimum_angular_coverage_ratio:=0.70 -p minimum_finite_return_ratio:=0.05 \
      -p maximum_revolution_duration_sec:=0.50 -p maximum_segment_gap_sec:=0.30 \
      -p maximum_input_messages_per_revolution:=1 -p publish_rate_limit_hz:=12.0 \
      -p diagnostics_rate_hz:=1.0 -p enable_deskew:=true \
      -p deskew_fixed_frame:="${ODOM_FRAME}" -p deskew_stamp_policy:="${DESKEW_STAMP_POLICY}" \
      -p deskew_timeout_sec:="${DESKEW_TIMEOUT_SEC}" -p use_sim_time:="${USE_SIM_TIME}" \
      -p health_csv_path:="${DESKEW_HEALTH_CSV}"
  DESKEW_PID="${LAST_STARTED_PID}"
  wait_for_message "${SCAN_TOPIC}" "${INPUT_WAIT_SEC}" best_effort "${DESKEW_PID}" "${DESKEW_LOG}"
  wait_for_diagnostic_true "${SCAN_DIAGNOSTICS_TOPIC}" deskew_applied \
    "${DESKEW_PID}" "${INPUT_WAIT_SEC}" "${DESKEW_LOG}"
  if [[ "$(publisher_count "${SCAN_TOPIC}")" != "1" ]]; then
    log_error "${SCAN_TOPIC} must have exactly one mapping-owned deskew publisher"
    ros2 topic info "${SCAN_TOPIC}" -v 2>/dev/null || true
    return 1
  fi
  log_started "mapping scan deskew active on ${SCAN_TOPIC}"
}

start_slam() {
  start_process slam_toolbox "${SLAM_LOG}" \
    ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:="${SLAM_PARAMS_FILE}" use_sim_time:="${USE_SIM_TIME}"
  SLAM_PID="${LAST_STARTED_PID}"
}

start_submap_slam() {
  [[ "${ENABLE_SUBMAP_SLAM}" == "1" ]] || return 0
  start_process submap_slam_2d "${SUBMAP_SLAM_LOG}" \
    ros2 launch submap_slam_2d submap_slam_2d.launch.py \
      params_file:="${SUBMAP_PARAMS_FILE}" use_sim_time:="${USE_SIM_TIME}" \
      scan_topic:="${SCAN_TOPIC}" full_odom_topic:="${ODOM_TOPIC}" \
      odom_frame:="${ODOM_FRAME}" base_frame:="${BASE_FRAME}" \
      map_topic:="${SUBMAP_MAP_TOPIC}" diagnostics_topic:="${SUBMAP_DIAGNOSTICS_TOPIC}"
  SUBMAP_SLAM_PID="${LAST_STARTED_PID}"
}

wait_for_submap_map() {
  [[ "${ENABLE_SUBMAP_SLAM}" == "1" ]] || return 0
  wait_for_message "${SUBMAP_DIAGNOSTICS_TOPIC}" "${MAP_WAIT_SEC}" reliable \
    "${SUBMAP_SLAM_PID}" "${SUBMAP_SLAM_LOG}"
  wait_for_message "${SUBMAP_MAP_TOPIC}" "${MAP_WAIT_SEC}" reliable \
    "${SUBMAP_SLAM_PID}" "${SUBMAP_SLAM_LOG}"
  log_started "enhanced 2D submap occupancy map ${SUBMAP_MAP_TOPIC}"
}

wait_for_map() {
  local start elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    if ! process_group_alive "${DESKEW_PID}"; then
      log_error "mapping scan deskew exited before ${MAP_TOPIC} became ready"
      tail -n 40 "${DESKEW_LOG}" 2>/dev/null || true
      return 1
    fi
    if ! process_group_alive "${SLAM_PID}"; then
      log_error "slam_toolbox exited before publishing ${MAP_TOPIC}"
      tail -n 60 "${SLAM_LOG}" 2>/dev/null || true
      return 1
    fi
    if timeout 3 ros2 topic echo "${MAP_TOPIC}" --once \
      --qos-reliability reliable --qos-durability transient_local >/dev/null 2>&1; then
      log_started "2D occupancy map ${MAP_TOPIC}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= MAP_WAIT_SEC )); then
      log_error "slam_toolbox is alive but no ${MAP_TOPIC} message arrived after ${MAP_WAIT_SEC}s"
      tail -n 60 "${SLAM_LOG}" 2>/dev/null || true
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${MAP_TOPIC}; elapsed=${elapsed}s deadline=${MAP_WAIT_SEC}s"
    fi
    sleep 1
  done
}

write_snapshot() {
  {
    printf 'timestamp=%s\nworkspace=%s\nuse_sim_time=%s\n' \
      "$(timestamp)" "${ROS_WS}" "${USE_SIM_TIME}"
    printf 'source_scan_topic=%s\ndeskewed_scan_topic=%s\ndeskew_stamp_policy=%s\n' \
      "${SOURCE_SCAN_TOPIC}" "${SCAN_TOPIC}" "${DESKEW_STAMP_POLICY}"
    printf 'map_topic=%s\nframes=%s->%s->%s->%s\n' \
      "${MAP_TOPIC}" "${MAP_FRAME}" "${ODOM_FRAME}" "${BASE_FRAME}" "${LIDAR_FRAME}"
    printf '\n-- deskew node --\n'
    timeout 5 ros2 node info /mapping_scan_deskew || true
    printf '\n-- source scan topic --\n'
    timeout 5 ros2 topic info "${SOURCE_SCAN_TOPIC}" -v || true
    printf '\n-- deskew diagnostics --\n'
    timeout 5 ros2 topic echo "${SCAN_DIAGNOSTICS_TOPIC}" --once || true
    printf '\n-- slam node --\n'
    timeout 5 ros2 node info /slam_toolbox || true
    if [[ "${ENABLE_SUBMAP_SLAM}" == "1" ]]; then
      printf '\n-- enhanced submap SLAM node --\n'
      timeout 5 ros2 node info /submap_slam_2d || true
      printf '\n-- enhanced submap diagnostics --\n'
      timeout 5 ros2 topic echo "${SUBMAP_DIAGNOSTICS_TOPIC}" --once || true
      printf '\n-- enhanced submap map topic --\n'
      timeout 5 ros2 topic info "${SUBMAP_MAP_TOPIC}" -v || true
    fi
    printf '\n-- map topic --\n'
    timeout 5 ros2 topic info "${MAP_TOPIC}" -v || true
    printf '\n-- scan topic --\n'
    timeout 5 ros2 topic info "${SCAN_TOPIC}" -v || true
  } >"${SNAPSHOT_FILE}" 2>&1
}

ready_banner() {
  printf '\n%s' "${GREEN}"
  printf '######################################################################\n'
  printf '#                                                                    #\n'
  printf '#                         2D MAP READY                               #\n'
  printf '#                                                                    #\n'
  printf '#          DESKEW ACTIVE - OBSERVER ONLY - NO FLIGHT CONTROL         #\n'
  printf '#                                                                    #\n'
  printf '######################################################################\n'
  printf '%s\n' "${RESET}"
}

main() {
  mkdir -p "${LOG_DIR}" "${MAP_SAVE_DIR}"
  touch "${MASTER_LOG}"
  # Keep the logger alive while the launcher handles Ctrl+C and terminates its
  # separately-grouped slam_toolbox child. It exits naturally when this pipe closes.
  exec > >(trap '' INT TERM; exec tee -a "${MASTER_LOG}") 2>&1
  trap cleanup EXIT
  trap 'on_signal INT' INT
  trap 'on_signal TERM' TERM

  if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "${ROS_SETUP}" ]]; then
    log_error "ROS setup is missing"
    exit 1
  fi
  if [[ ! -f "${SLAM_PARAMS_FILE}" ]]; then
    log_error "SLAM configuration is missing: ${SLAM_PARAMS_FILE}"
    exit 1
  fi

  set +u
  source /opt/ros/jazzy/setup.bash
  source "${ROS_SETUP}"
  set -u

  for command in ros2 timeout setsid flock awk grep tee; do
    if ! command -v "${command}" >/dev/null 2>&1; then
      log_error "missing command: ${command}"
      exit 1
    fi
  done

  validate_launcher_settings

  log "Independent observer-only 2D mapping startup"
  log "This launcher starts mapping scan deskew, slam_toolbox, and enhanced submap SLAM"
  log "It will not manage RF2O/PX4 fusion, MAVROS, LiDAR, camera, AprilTag, planner, or flight control"

  acquire_mapping_lock
  require_clean_mapping_namespace
  capture_control_baseline

  if [[ "$(publisher_count "${SOURCE_SCAN_TOPIC}")" != "1" ]]; then
    log_error "${SOURCE_SCAN_TOPIC} must have exactly one publisher from the fusion pipeline"
    ros2 topic info "${SOURCE_SCAN_TOPIC}" -v 2>/dev/null || true
    exit 1
  fi
  if [[ "${USE_SIM_TIME}" == "true" ]]; then
    wait_for_message /clock "${INPUT_WAIT_SEC}" best_effort
  fi
  wait_for_message "${SOURCE_SCAN_TOPIC}" "${INPUT_WAIT_SEC}" best_effort
  wait_for_message "${ODOM_TOPIC}" "${INPUT_WAIT_SEC}" best_effort
  verify_live_timestamp_domain "${SOURCE_SCAN_TOPIC}"
  wait_for_transform "${ODOM_FRAME}" "${BASE_FRAME}" "${INPUT_WAIT_SEC}"
  wait_for_transform "${BASE_FRAME}" "${LIDAR_FRAME}" "${INPUT_WAIT_SEC}"

  start_deskew
  verify_live_timestamp_domain "${SCAN_TOPIC}"
  start_slam
  start_submap_slam
  wait_for_map
  wait_for_submap_map
  wait_for_transform "${MAP_FRAME}" "${ODOM_FRAME}" "${INPUT_WAIT_SEC}"
  wait_for_transform "${MAP_FRAME}" "${LIDAR_FRAME}" "${INPUT_WAIT_SEC}"
  validate_slam_node
  verify_no_control_publishers_added
  write_snapshot
  ready_banner

  log "Move the vehicle manually to build the map; this process sends no vehicle commands"
  log "On Ctrl+C, autosave writes ${MAP_SAVE_DIR}/${MAP_SAVE_PREFIX}_${RUN_STAMP}.yaml/.pgm/.png"
  log "Runtime logs: ${LOG_DIR}"

  while process_group_alive "${SLAM_PID}" && process_group_alive "${DESKEW_PID}" && \
    { [[ "${ENABLE_SUBMAP_SLAM}" == "0" ]] || process_group_alive "${SUBMAP_SLAM_PID}"; }; do
    sleep 2
  done
  if ! process_group_alive "${DESKEW_PID}"; then
    SHUTDOWN_REASON="mapping scan deskew exited unexpectedly"
    tail -n 40 "${DESKEW_LOG}" 2>/dev/null || true
  elif ! process_group_alive "${SLAM_PID}"; then
    SHUTDOWN_REASON="slam_toolbox exited unexpectedly"
    tail -n 40 "${SLAM_LOG}" 2>/dev/null || true
  else
    SHUTDOWN_REASON="enhanced submap SLAM exited unexpectedly"
    tail -n 60 "${SUBMAP_SLAM_LOG}" 2>/dev/null || true
  fi
  log_error "${SHUTDOWN_REASON}"
  return 1
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
