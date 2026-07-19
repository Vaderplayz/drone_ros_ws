#!/usr/bin/env bash
# Real-drone lidar2 vertical 3D mapping.
# Starts only the second RPLIDAR and vertical_lidar_mapper. It does not touch
# lidar1, RF2O, MAVROS, AprilTag, planner, arming, mode, or setpoint topics.
# shellcheck disable=SC1090,SC1091

set -euo pipefail

ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

LIDAR2_SERIAL_PORT="${LIDAR2_SERIAL_PORT:-/dev/ttyUSB1}"
LIDAR2_BAUDRATE="${LIDAR2_BAUDRATE:-460800}"
LIDAR2_FRAME_ID="${LIDAR2_FRAME_ID:-lidar_vert_link}"
LIDAR2_SCAN_TOPIC="${LIDAR2_SCAN_TOPIC:-/scan_vertical}"
LIDAR2_NODE_NAME="${LIDAR2_NODE_NAME:-sllidar2_vertical}"
LIDAR2_SCAN_WAIT_SEC="${LIDAR2_SCAN_WAIT_SEC:-60}"
LIDAR2_INVERTED="${LIDAR2_INVERTED:-false}"
LIDAR2_ANGLE_COMPENSATE="${LIDAR2_ANGLE_COMPENSATE:-true}"
LIDAR2_SCAN_MODE="${LIDAR2_SCAN_MODE:-Standard}"

ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
TARGET_FRAME="${TARGET_FRAME:-map}"
MAP_FRAME="${MAP_FRAME:-map}"
PX4_ODOM_TOPIC="${PX4_ODOM_TOPIC:-/mavros/local_position/odom}"
REQUIRE_2D_MAP="${REQUIRE_2D_MAP:-1}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-90}"
POINTCLOUD_WAIT_SEC="${POINTCLOUD_WAIT_SEC:-90}"

# Mount convention:
# - ROS base frame is x-forward, y-left, z-up.
# - The C1M1 local +X ("forward") points to the drone's left (+Y).
# - The scan plane is vertical, with local +Y pointing up.
START_LIDAR2_STATIC_TF="${START_LIDAR2_STATIC_TF:-1}"
LIDAR2_X="${LIDAR2_X:-0.0}"
LIDAR2_Y="${LIDAR2_Y:-0.0}"
LIDAR2_Z="${LIDAR2_Z:-0.70}"
LIDAR2_ROLL="${LIDAR2_ROLL:-1.57079632679}"
LIDAR2_PITCH="${LIDAR2_PITCH:-0.0}"
LIDAR2_YAW="${LIDAR2_YAW:-1.57079632679}"

MAPPER_PARAMS_FILE="${MAPPER_PARAMS_FILE:-${ROS_WS}/src/vertical_lidar_mapper/config/real_c1m1_left.yaml}"
VERTICAL_CLOUD_TOPIC="${VERTICAL_CLOUD_TOPIC:-/vertical_cloud}"
DESKEWED_CLOUD_TOPIC="${DESKEWED_CLOUD_TOPIC:-/vertical_points_deskewed}"
VERTICAL_MAP_TOPIC="${VERTICAL_MAP_TOPIC:-/vertical_map}"
GLOBAL_CLOUD_TOPIC="${GLOBAL_CLOUD_TOPIC:-/mapping/global_cloud}"
MAPPING_STATUS_TOPIC="${MAPPING_STATUS_TOPIC:-/mapping/status}"
EXPORT_DIR="${EXPORT_DIR:-${ROS_WS}/maps/vertical_3d}"
RECORD_VERTICAL_BAG="${RECORD_VERTICAL_BAG:-0}"
AUTO_SAVE_3D_MAP_ON_EXIT="${AUTO_SAVE_3D_MAP_ON_EXIT:-1}"
SAVE_SERVICE_TIMEOUT_SEC="${SAVE_SERVICE_TIMEOUT_SEC:-30}"
PROCESS_STOP_TIMEOUT_SEC="${PROCESS_STOP_TIMEOUT_SEC:-5}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROS_WS}/runtime_logs/lidar2_3d_${RUN_STAMP}"
MASTER_LOG="${LOG_DIR}/master.log"
LIDAR2_LOG="${LOG_DIR}/sllidar2.log"
STATIC_TF_LOG="${LOG_DIR}/lidar2_static_tf.log"
MAPPER_LOG="${LOG_DIR}/vertical_lidar_mapper.log"
ROSBAG_LOG="${LOG_DIR}/vertical_mapping_bag.log"
SNAPSHOT_FILE="${LOG_DIR}/system_snapshot.txt"

STATE_ROOT="${XDG_RUNTIME_DIR:-/tmp}/lidar2_3d_mapping_$(id -u)"
LOCK_FILE="${STATE_ROOT}/launcher.lock"
OWNER_FILE="${STATE_ROOT}/launcher.pid"
OWNED_GROUPS_FILE="${STATE_ROOT}/owned_process_groups.tsv"
BOOT_ID="$(< /proc/sys/kernel/random/boot_id)"

PIDS=()
NAMES=()
LAST_STARTED_PID=""
LOCK_ACQUIRED=0
EXTERNAL_LIDAR2=0
EXTERNAL_LIDAR2_TF=0
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

process_group_alive() {
  kill -0 -- "-$1" 2>/dev/null || kill -0 "$1" 2>/dev/null
}

signal_process_group() {
  local pid="$1" signal="$2"
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  elif kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

add_process() {
  local pid="$1" name="$2"
  PIDS+=("${pid}")
  NAMES+=("${name}")
  printf '%s\n' "${pid}" >"${LOG_DIR}/${name}.pid"
  printf '%s\t%s\t%s\n' "${pid}" "${name}" "${BOOT_ID}" >>"${OWNED_GROUPS_FILE}"
  log_started "${name} pgid=${pid}"
}

start_process() {
  local name="$1" logfile="$2"
  shift 2
  setsid "$@" 9>&- >"${logfile}" 2>&1 &
  LAST_STARTED_PID="$!"
  add_process "${LAST_STARTED_PID}" "${name}"
}

stop_owned_groups() {
  local index pid deadline any_alive
  for ((index=${#PIDS[@]} - 1; index>=0; index--)); do
    pid="${PIDS[$index]}"
    if process_group_alive "${pid}"; then
      log "Stopping launcher-owned process group: ${NAMES[$index]} pgid=${pid}"
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

save_3d_map_on_exit() {
  if [[ "${AUTO_SAVE_3D_MAP_ON_EXIT}" != "1" ]]; then
    log "3D map autosave disabled by AUTO_SAVE_3D_MAP_ON_EXIT=${AUTO_SAVE_3D_MAP_ON_EXIT}"
    return 0
  fi
  if ! ros2 service list 2>/dev/null | grep -qx '/vertical_lidar_mapper/save_pcd'; then
    log_warning "3D map autosave skipped because /vertical_lidar_mapper/save_pcd is unavailable"
    return 0
  fi
  mkdir -p "${EXPORT_DIR}"
  log "Autosaving 3D map assets through /vertical_lidar_mapper/save_pcd"
  if timeout "${SAVE_SERVICE_TIMEOUT_SEC}" ros2 service call \
    /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}" >/dev/null 2>&1; then
    log "3D map autosave requested; PCD/GLB output directory: ${EXPORT_DIR}"
  else
    log_warning "3D map autosave service call did not complete within ${SAVE_SERVICE_TIMEOUT_SEC}s"
  fi
}

cleanup() {
  local exit_code=$? pid
  set +e
  trap - EXIT INT TERM
  log "Shutdown reason: ${SHUTDOWN_REASON}; launcher_exit=${exit_code}"
  save_3d_map_on_exit
  stop_owned_groups
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  if [[ "${LOCK_ACQUIRED}" == "1" ]]; then
    : >"${OWNED_GROUPS_FILE}"
    : >"${OWNER_FILE}"
  fi
  if [[ "${EXTERNAL_LIDAR2}" == "1" ]]; then
    log "Externally managed lidar2 publisher was not stopped"
  fi
  if [[ "${EXTERNAL_LIDAR2_TF}" == "1" ]]; then
    log "Externally managed lidar2 static TF was not stopped"
  fi
  log "lidar1, RF2O, 2D SLAM, MAVROS, camera, AprilTag, planner, and flight control were not stopped"
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 130
}

recover_recorded_groups() {
  local pid name recorded_boot deadline any_alive
  local recovered_pids=()
  [[ -s "${OWNED_GROUPS_FILE}" ]] || return 0

  while IFS=$'\t' read -r pid name recorded_boot; do
    [[ "${pid:-}" =~ ^[0-9]+$ ]] || continue
    [[ "${recorded_boot:-}" == "${BOOT_ID}" ]] || continue
    if process_group_alive "${pid}"; then
      recovered_pids+=("${pid}")
    fi
  done <"${OWNED_GROUPS_FILE}"

  (( ${#recovered_pids[@]} == 0 )) && return 0

  log_warning "recovering ${#recovered_pids[@]} lidar2 mapping process group(s) from an interrupted run"
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
  log_started "previous lidar2 mapping-owned process groups cleared"
}

acquire_launcher_lock() {
  mkdir -p "${STATE_ROOT}"
  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    log_error "another lidar2 3D mapping launcher owns ${LOCK_FILE}"
    [[ -s "${OWNER_FILE}" ]] && log_error "launcher pid=$(<"${OWNER_FILE}")"
    return 1
  fi
  LOCK_ACQUIRED=1
  printf '%s\n' "$$" >"${OWNER_FILE}"
  recover_recorded_groups
  : >"${OWNED_GROUPS_FILE}"
}

validate_settings() {
  local topic frame number
  case "${USE_SIM_TIME}" in
    true|false) ;;
    *) log_error "USE_SIM_TIME must be true or false, got '${USE_SIM_TIME}'"; return 1 ;;
  esac
  case "${REQUIRE_2D_MAP}" in
    0|1) ;;
    *) log_error "REQUIRE_2D_MAP must be 0 or 1, got '${REQUIRE_2D_MAP}'"; return 1 ;;
  esac
  for topic in "${LIDAR2_SCAN_TOPIC}" "${DESKEWED_CLOUD_TOPIC}" "${VERTICAL_CLOUD_TOPIC}" "${VERTICAL_MAP_TOPIC}" \
    "${GLOBAL_CLOUD_TOPIC}" "${MAPPING_STATUS_TOPIC}" "${PX4_ODOM_TOPIC}"; do
    if [[ ! "${topic}" =~ ^/[A-Za-z0-9_/]+$ || "${topic}" == *//* || "${topic}" == */ ]]; then
      log_error "invalid absolute ROS topic name '${topic}'"
      return 1
    fi
  done
  for frame in "${ODOM_FRAME}" "${BASE_FRAME}" "${TARGET_FRAME}" "${MAP_FRAME}" "${LIDAR2_FRAME_ID}"; do
    if [[ ! "${frame}" =~ ^[A-Za-z][A-Za-z0-9_/]*$ || "${frame}" == *//* || "${frame}" == */ ]]; then
      log_error "invalid TF frame name '${frame}'"
      return 1
    fi
  done
  for number in "${LIDAR2_X}" "${LIDAR2_Y}" "${LIDAR2_Z}" \
    "${LIDAR2_ROLL}" "${LIDAR2_PITCH}" "${LIDAR2_YAW}"; do
    if [[ ! "${number}" =~ ^-?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
      log_error "lidar2 pose values must be numeric, got '${number}'"
      return 1
    fi
  done
  if [[ ! -f "${MAPPER_PARAMS_FILE}" ]]; then
    log_error "mapper params file not found: ${MAPPER_PARAMS_FILE}"
    return 1
  fi
}

wait_for_message() {
  local topic="$1" timeout_sec="$2" reliability="$3" pid="${4:-}" logfile="${5:-}"
  local start elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    if [[ -n "${pid}" ]] && ! kill -0 "${pid}" 2>/dev/null; then
      log_error "process pid=${pid} exited before publishing ${topic}"
      if [[ -n "${logfile}" ]]; then
        tail -n 60 "${logfile}" 2>/dev/null || true
      fi
      return 10
    fi
    if timeout 3 ros2 topic echo "${topic}" --once \
      --qos-reliability "${reliability}" >/dev/null 2>&1; then
      log_started "message stream ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "no message on ${topic} after ${timeout_sec}s"
      if [[ -n "${logfile}" ]]; then
        tail -n 60 "${logfile}" 2>/dev/null || true
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

tf_available() {
  local parent="$1" child="$2" output
  output="$(timeout 3 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
  grep -q -- '- Translation:' <<<"${output}"
}

wait_for_transform() {
  local parent="$1" child="$2" timeout_sec="$3" pid="${4:-}" logfile="${5:-}"
  local start elapsed output last_progress=-1
  start="$(date +%s)"
  while true; do
    if [[ -n "${pid}" ]] && ! kill -0 "${pid}" 2>/dev/null; then
      log_error "process pid=${pid} exited while waiting for TF ${parent} -> ${child}"
      if [[ -n "${logfile}" ]]; then
        tail -n 60 "${logfile}" 2>/dev/null || true
      fi
      return 10
    fi
    output="$(timeout 3 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
    if grep -q -- '- Translation:' <<<"${output}"; then
      log_started "TF ${parent} -> ${child}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "missing TF ${parent} -> ${child} after ${timeout_sec}s"
      if [[ -n "${logfile}" ]]; then
        tail -n 60 "${logfile}" 2>/dev/null || true
      fi
      return 12
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for TF ${parent} -> ${child}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

wait_for_2d_map_if_required() {
  if [[ "${REQUIRE_2D_MAP}" != "1" ]]; then
    log_warning "REQUIRE_2D_MAP=0; mapper will use TARGET_FRAME=${TARGET_FRAME}"
    return 0
  fi
  if [[ "${TARGET_FRAME}" != "${MAP_FRAME}" ]]; then
    log_warning "REQUIRE_2D_MAP=1 but TARGET_FRAME=${TARGET_FRAME}; still checking map inputs"
  fi
  wait_for_message /map "${WAIT_TIMEOUT_SEC}" reliable
  wait_for_transform "${MAP_FRAME}" "${ODOM_FRAME}" "${WAIT_TIMEOUT_SEC}"
}

lidar_process_using_lidar2_port() {
  pgrep -af '[s]llidar_node|[r]plidar_composition|[r]plidar_node' 2>/dev/null | \
    grep -F "serial_port:=${LIDAR2_SERIAL_PORT}" || true
}

start_lidar2_driver() {
  local existing_publishers pid process_on_port
  existing_publishers="$(publisher_count "${LIDAR2_SCAN_TOPIC}")"
  if [[ "${existing_publishers}" == "1" ]]; then
    wait_for_message "${LIDAR2_SCAN_TOPIC}" 10 best_effort
    EXTERNAL_LIDAR2=1
    log_started "reusing existing lidar2 publisher on ${LIDAR2_SCAN_TOPIC}"
    return 0
  fi
  if [[ "${existing_publishers}" != "0" ]]; then
    log_error "${LIDAR2_SCAN_TOPIC} publisher_count=${existing_publishers}, expected 0 or 1"
    ros2 topic info "${LIDAR2_SCAN_TOPIC}" -v 2>/dev/null || true
    return 1
  fi
  process_on_port="$(lidar_process_using_lidar2_port)"
  if [[ -n "${process_on_port}" ]]; then
    log_error "a lidar process already uses ${LIDAR2_SERIAL_PORT}, but ${LIDAR2_SCAN_TOPIC} has no publisher"
    printf '%s\n' "${process_on_port}"
    return 1
  fi
  if [[ ! -e "${LIDAR2_SERIAL_PORT}" || ! -r "${LIDAR2_SERIAL_PORT}" ||
    ! -w "${LIDAR2_SERIAL_PORT}" ]]; then
    log_error "${LIDAR2_SERIAL_PORT} is absent or not readable/writable by $(id -un)"
    return 1
  fi

  start_process sllidar2 "${LIDAR2_LOG}" \
    ros2 run sllidar_ros2 sllidar_node --ros-args \
      -r __node:="${LIDAR2_NODE_NAME}" \
      -r scan:="${LIDAR2_SCAN_TOPIC}" \
      -p channel_type:=serial \
      -p serial_port:="${LIDAR2_SERIAL_PORT}" \
      -p serial_baudrate:="${LIDAR2_BAUDRATE}" \
      -p frame_id:="${LIDAR2_FRAME_ID}" \
      -p inverted:="${LIDAR2_INVERTED}" \
      -p angle_compensate:="${LIDAR2_ANGLE_COMPENSATE}" \
      -p scan_mode:="${LIDAR2_SCAN_MODE}" \
      -p use_sim_time:="${USE_SIM_TIME}"
  pid="${LAST_STARTED_PID}"
  wait_for_message "${LIDAR2_SCAN_TOPIC}" "${LIDAR2_SCAN_WAIT_SEC}" best_effort "${pid}" "${LIDAR2_LOG}"
  if [[ "$(publisher_count "${LIDAR2_SCAN_TOPIC}")" != "1" ]]; then
    log_error "${LIDAR2_SCAN_TOPIC} must have exactly one publisher"
    ros2 topic info "${LIDAR2_SCAN_TOPIC}" -v 2>/dev/null || true
    return 1
  fi
}

start_lidar2_static_tf() {
  local pid
  if tf_available "${BASE_FRAME}" "${LIDAR2_FRAME_ID}"; then
    EXTERNAL_LIDAR2_TF=1
    log_started "reusing existing TF ${BASE_FRAME} -> ${LIDAR2_FRAME_ID}"
    return 0
  fi
  if [[ "${START_LIDAR2_STATIC_TF}" != "1" ]]; then
    log_error "missing TF ${BASE_FRAME} -> ${LIDAR2_FRAME_ID} and START_LIDAR2_STATIC_TF=${START_LIDAR2_STATIC_TF}"
    return 1
  fi
  start_process lidar2_static_tf "${STATIC_TF_LOG}" \
    ros2 run tf2_ros static_transform_publisher \
      --x "${LIDAR2_X}" --y "${LIDAR2_Y}" --z "${LIDAR2_Z}" \
      --roll "${LIDAR2_ROLL}" --pitch "${LIDAR2_PITCH}" --yaw "${LIDAR2_YAW}" \
      --frame-id "${BASE_FRAME}" --child-frame-id "${LIDAR2_FRAME_ID}" \
      --ros-args -p use_sim_time:="${USE_SIM_TIME}"
  pid="${LAST_STARTED_PID}"
  wait_for_transform "${BASE_FRAME}" "${LIDAR2_FRAME_ID}" "${WAIT_TIMEOUT_SEC}" "${pid}" "${STATIC_TF_LOG}"
}

require_mapper_namespace_free() {
  local nodes topic count
  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -qx '/vertical_lidar_mapper' <<<"${nodes}"; then
    log_error "/vertical_lidar_mapper is already running"
    return 1
  fi
  for topic in "${DESKEWED_CLOUD_TOPIC}" "${VERTICAL_CLOUD_TOPIC}" "${VERTICAL_MAP_TOPIC}" "${GLOBAL_CLOUD_TOPIC}" \
    "${MAPPING_STATUS_TOPIC}"; do
    count="$(publisher_count "${topic}")"
    if [[ "${count}" != "0" ]]; then
      log_error "${topic} already has ${count} publisher(s); refusing a duplicate mapper"
      ros2 topic info "${topic}" -v 2>/dev/null || true
      return 1
    fi
  done
}

start_mapper() {
  local pid
  mkdir -p "${EXPORT_DIR}"
  require_mapper_namespace_free
  start_process vertical_lidar_mapper "${MAPPER_LOG}" \
    ros2 run vertical_lidar_mapper vertical_lidar_mapper_node --ros-args \
      --params-file "${MAPPER_PARAMS_FILE}" \
      -p use_sim_time:="${USE_SIM_TIME}" \
      -p scan_topic:="${LIDAR2_SCAN_TOPIC}" \
      -p target_frame:="${TARGET_FRAME}" \
      -p base_frame:="${BASE_FRAME}" \
      -p lidar_frame_override:="${LIDAR2_FRAME_ID}" \
      -p motion_odom_topic:="${PX4_ODOM_TOPIC}" \
      -p motion_odom_frame:="${ODOM_FRAME}" \
      -p autosave_on_exit:=false \
      -p pcd_export_dir:="${EXPORT_DIR}" \
      -p map_rebase_map_frame:="${MAP_FRAME}" \
      -p map_rebase_odom_frame:="${ODOM_FRAME}" \
      -p relative_pose_map_frame:="${MAP_FRAME}" \
      -p relative_pose_odom_frame:="${ODOM_FRAME}"
  pid="${LAST_STARTED_PID}"
  wait_for_message "${VERTICAL_CLOUD_TOPIC}" "${POINTCLOUD_WAIT_SEC}" best_effort "${pid}" "${MAPPER_LOG}"
  wait_for_message "${GLOBAL_CLOUD_TOPIC}" "${POINTCLOUD_WAIT_SEC}" reliable "${pid}" "${MAPPER_LOG}"
}

start_vertical_bag() {
  if [[ "${RECORD_VERTICAL_BAG}" != "1" ]]; then
    return 0
  fi
  start_process vertical_mapping_bag "${ROSBAG_LOG}" \
    ros2 bag record -o "${LOG_DIR}/vertical_mapping_bag" \
      "${LIDAR2_SCAN_TOPIC}" "${DESKEWED_CLOUD_TOPIC}" "${VERTICAL_CLOUD_TOPIC}" "${VERTICAL_MAP_TOPIC}" \
      "${GLOBAL_CLOUD_TOPIC}" "${MAPPING_STATUS_TOPIC}" "${PX4_ODOM_TOPIC}" \
      /map /tf /tf_static
}

write_snapshot() {
  {
    printf 'timestamp=%s\nworkspace=%s\nuse_sim_time=%s\n' \
      "$(timestamp)" "${ROS_WS}" "${USE_SIM_TIME}"
    printf 'lidar2_port=%s\nlidar2_baudrate=%s\nlidar2_scan_topic=%s\nlidar2_frame=%s\n' \
      "${LIDAR2_SERIAL_PORT}" "${LIDAR2_BAUDRATE}" "${LIDAR2_SCAN_TOPIC}" "${LIDAR2_FRAME_ID}"
    printf 'frames=%s->%s->%s->%s\n' "${TARGET_FRAME}" "${ODOM_FRAME}" "${BASE_FRAME}" "${LIDAR2_FRAME_ID}"
    printf 'lidar2_static_tf_xyz_rpy=%s,%s,%s,%s,%s,%s\n' \
      "${LIDAR2_X}" "${LIDAR2_Y}" "${LIDAR2_Z}" "${LIDAR2_ROLL}" "${LIDAR2_PITCH}" "${LIDAR2_YAW}"
    printf 'mapper_params=%s\nexport_dir=%s\n' "${MAPPER_PARAMS_FILE}" "${EXPORT_DIR}"
    for topic in "${LIDAR2_SCAN_TOPIC}" "${DESKEWED_CLOUD_TOPIC}" "${VERTICAL_CLOUD_TOPIC}" "${VERTICAL_MAP_TOPIC}" \
      "${GLOBAL_CLOUD_TOPIC}" "${MAPPING_STATUS_TOPIC}"; do
      printf '\n-- %s --\n' "${topic}"
      ros2 topic info "${topic}" -v || true
    done
    printf '\n-- mapper status --\n'
    timeout 5 ros2 topic echo "${MAPPING_STATUS_TOPIC}" --once || true
    printf '\n-- mapper node --\n'
    timeout 5 ros2 node info /vertical_lidar_mapper || true
    printf '\n-- TF checks --\n'
    timeout 5 ros2 run tf2_ros tf2_echo "${BASE_FRAME}" "${LIDAR2_FRAME_ID}" || true
    timeout 5 ros2 run tf2_ros tf2_echo "${TARGET_FRAME}" "${LIDAR2_FRAME_ID}" || true
  } >"${SNAPSHOT_FILE}" 2>&1
}

ready_banner() {
  printf '\n%s' "${GREEN}"
  printf '######################################################################\n'
  printf '#                                                                    #\n'
  printf '#                       LIDAR2 3D MAP READY                          #\n'
  printf '#                                                                    #\n'
  printf '#             VERTICAL C1M1 MAPPING - OBSERVER ONLY                  #\n'
  printf '#                                                                    #\n'
  printf '######################################################################\n'
  printf '%s\n' "${RESET}"
}

main() {
  mkdir -p "${LOG_DIR}"
  touch "${MASTER_LOG}"
  exec > >(trap '' INT TERM; exec tee -a "${MASTER_LOG}") 2>&1
  trap cleanup EXIT
  trap 'on_signal INT' INT
  trap 'on_signal TERM' TERM

  if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "${ROS_SETUP}" ]]; then
    log_error "ROS setup is missing"
    exit 1
  fi

  set +u
  source /opt/ros/jazzy/setup.bash
  source "${ROS_SETUP}"
  set -u

  for command in ros2 timeout pgrep setsid flock awk grep mkdir tee; do
    if ! command -v "${command}" >/dev/null 2>&1; then
      log_error "missing command: ${command}"
      exit 1
    fi
  done
  if ! ros2 pkg prefix sllidar_ros2 >/dev/null 2>&1; then
    log_error "sllidar_ros2 is unavailable; install/build Slamtec's C1-compatible ROS 2 driver"
    exit 1
  fi
  if ! ros2 pkg prefix vertical_lidar_mapper >/dev/null 2>&1; then
    log_error "vertical_lidar_mapper package is unavailable in this overlay"
    exit 1
  fi

  validate_settings
  acquire_launcher_lock

  log "Real lidar2 3D mapping startup"
  log "Workspace=${ROS_WS}; lidar2=${LIDAR2_SERIAL_PORT}@${LIDAR2_BAUDRATE}; target_frame=${TARGET_FRAME}"
  log "This launcher will not manage lidar1, RF2O, 2D SLAM, MAVROS, or flight control"

  wait_for_message "${PX4_ODOM_TOPIC}" "${WAIT_TIMEOUT_SEC}" best_effort
  wait_for_transform "${ODOM_FRAME}" "${BASE_FRAME}" "${WAIT_TIMEOUT_SEC}"
  wait_for_2d_map_if_required
  start_lidar2_driver
  start_lidar2_static_tf
  wait_for_transform "${TARGET_FRAME}" "${LIDAR2_FRAME_ID}" "${WAIT_TIMEOUT_SEC}"
  start_vertical_bag
  start_mapper
  write_snapshot
  ready_banner

  log "Export service: ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger '{}'"
  log "On Ctrl+C, autosave requests one PCD/GLB asset set in ${EXPORT_DIR}"
  log "Runtime logs: ${LOG_DIR}"
  log "No PX4 parameter, arm, mode, movement, or setpoint command was sent"

  while true; do
    local index
    for index in "${!PIDS[@]}"; do
      if ! kill -0 "${PIDS[$index]}" 2>/dev/null; then
        SHUTDOWN_REASON="required process exited: ${NAMES[$index]} pid=${PIDS[$index]}"
        log_error "${SHUTDOWN_REASON}"
        return 1
      fi
    done
    if [[ "${EXTERNAL_LIDAR2}" == "1" ]]; then
      wait_for_message "${LIDAR2_SCAN_TOPIC}" 5 best_effort >/dev/null || {
        SHUTDOWN_REASON="external lidar2 scan stream stopped"
        return 1
      }
    fi
    sleep 2
  done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
