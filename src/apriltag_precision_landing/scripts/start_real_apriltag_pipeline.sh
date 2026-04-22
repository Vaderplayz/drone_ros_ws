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

kill_if_running() {
  local pattern="$1"
  pgrep -af "${pattern}" >/dev/null 2>&1 || return 0
  pkill -f "${pattern}" >/dev/null 2>&1 || true
}

resolve_ros_setup() {
  local candidates=()

  if [[ -n "${ROS_SETUP:-}" ]]; then
    candidates+=("${ROS_SETUP}")
  fi
  if [[ -n "${ROS_WS:-}" ]]; then
    candidates+=("${ROS_WS}/install/setup.bash")
  fi

  # Running from source tree: <ws>/src/apriltag_precision_landing/scripts
  candidates+=("${SCRIPT_DIR}/../../../install/setup.bash")
  # Running from install tree: <ws>/install/apriltag_precision_landing/lib/apriltag_precision_landing
  candidates+=("${SCRIPT_DIR}/../../../../setup.bash")
  candidates+=("${SCRIPT_DIR}/../../../../local_setup.bash")
  # Fallback to current directory workspace layout.
  candidates+=("${PWD}/install/setup.bash")

  local c
  for c in "${candidates[@]}"; do
    if [[ -f "${c}" ]]; then
      (cd "$(dirname "${c}")" >/dev/null 2>&1 && printf "%s/%s\n" "$(pwd)" "$(basename "${c}")") || continue
      return 0
    fi
  done
  return 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if ! ROS_SETUP="$(resolve_ros_setup)"; then
  ROS_SETUP="${SCRIPT_DIR}/../../../install/setup.bash"
fi
ROS_WS="${ROS_WS:-$(cd "$(dirname "${ROS_SETUP}")/.." >/dev/null 2>&1 && pwd || true)}"

KILL_BEFORE_LAUNCH="${KILL_BEFORE_LAUNCH:-1}"

START_MAVROS="${START_MAVROS:-1}"
START_CAMERA="${START_CAMERA:-0}"
START_IMAGE_VIEW="${START_IMAGE_VIEW:-0}"
DETECTOR_INPUT_SOURCE="${DETECTOR_INPUT_SOURCE:-device}"

MAVROS_LAUNCH_FILE="${MAVROS_LAUNCH_FILE:-px4.launch}"
FCU_URL="${FCU_URL:-serial:///dev/ttyACM0:115200}"
MAVROS_RESPAWN="${MAVROS_RESPAWN:-true}"

VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
OUTPUT_ENCODING="${OUTPUT_ENCODING:-rgb8}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera_info}"
CAMERA_FRAME_ID="${CAMERA_FRAME_ID:-camera_link}"
DETECTOR_DEVICE_WIDTH="${DETECTOR_DEVICE_WIDTH:-640}"
DETECTOR_DEVICE_HEIGHT="${DETECTOR_DEVICE_HEIGHT:-480}"
DETECTOR_DEVICE_FPS="${DETECTOR_DEVICE_FPS:-30.0}"

TAG_DICTIONARY="${TAG_DICTIONARY:-36h11}"
TAG_SIZE_M="${TAG_SIZE_M:-0.16}"
TARGET_TAG_ID="${TARGET_TAG_ID:--1}"
MIN_TAG_AREA_PX="${MIN_TAG_AREA_PX:-40.0}"
TAG_POSE_TOPIC="${TAG_POSE_TOPIC:-/precision_landing/tag_pose_camera}"

DRONE_POSE_TOPIC="${DRONE_POSE_TOPIC:-/mavros/local_position/pose}"
LANDING_TARGET_TOPIC="${LANDING_TARGET_TOPIC:-/mavros/landing_target/pose}"
WORLD_FRAME="${WORLD_FRAME:-map}"
INPUT_TIMEOUT_SEC="${INPUT_TIMEOUT_SEC:-0.30}"

CAMERA_OFFSET_X="${CAMERA_OFFSET_X:-0.0}"
CAMERA_OFFSET_Y="${CAMERA_OFFSET_Y:-0.0}"
CAMERA_OFFSET_Z="${CAMERA_OFFSET_Z:-0.0}"
CAMERA_ROLL="${CAMERA_ROLL:-0.0}"
CAMERA_PITCH="${CAMERA_PITCH:-3.141592653589793}"
CAMERA_YAW="${CAMERA_YAW:-1.5707963267948966}"

PUBLISH_DEBUG_TF="${PUBLISH_DEBUG_TF:-true}"
DEBUG_TAG_CHILD_FRAME="${DEBUG_TAG_CHILD_FRAME:-apriltag_detected}"

MAVROS_LOG="${MAVROS_LOG:-/tmp/mavros_apriltag.log}"
CAMERA_LOG="${CAMERA_LOG:-/tmp/v4l2_camera_apriltag.log}"
DETECTOR_LOG="${DETECTOR_LOG:-/tmp/apriltag_detector.log}"
LANDING_LOG="${LANDING_LOG:-/tmp/apriltag_landing_target.log}"
IMAGE_VIEW_LOG="${IMAGE_VIEW_LOG:-/tmp/rqt_image_view_apriltag.log}"

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
  kill_if_running "/mavros_node"
  kill_if_running "ros2 run v4l2_camera v4l2_camera_node"
  kill_if_running "ros2 run apriltag_precision_landing apriltag_camera_detector_node"
  kill_if_running "ros2 run apriltag_precision_landing apriltag_precision_landing_node"
  kill_if_running "ros2 run rqt_image_view rqt_image_view"
}

start_mavros() {
  echo "[run] MAVROS -> ${MAVROS_LOG}"
  ros2 launch mavros "${MAVROS_LAUNCH_FILE}" \
    fcu_url:="${FCU_URL}" \
    respawn_mavros:="${MAVROS_RESPAWN}" \
    use_sim_time:=false >"${MAVROS_LOG}" 2>&1 &
  add_process "$!" "mavros"
}

start_camera() {
  echo "[run] v4l2_camera (${VIDEO_DEVICE}) -> ${CAMERA_LOG}"
  ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:="${VIDEO_DEVICE}" \
    -p output_encoding:="${OUTPUT_ENCODING}" \
    -r /image_raw:="${IMAGE_TOPIC}" \
    -r /camera_info:="${CAMERA_INFO_TOPIC}" >"${CAMERA_LOG}" 2>&1 &
  add_process "$!" "v4l2_camera"
}

start_detector() {
  local source="${DETECTOR_INPUT_SOURCE}"
  if [[ "${source}" == "auto" ]]; then
    source="device"
  fi

  if [[ "${source}" != "ros_topics" && "${source}" != "device" ]]; then
    echo "[error] invalid DETECTOR_INPUT_SOURCE='${DETECTOR_INPUT_SOURCE}' (use auto|ros_topics|device)" >&2
    exit 1
  fi

  echo "[run] apriltag_camera_detector_node -> ${DETECTOR_LOG}"
  local cmd=(ros2 run apriltag_precision_landing apriltag_camera_detector_node --ros-args
    -p "input_source:=${source}"
    -p "image_topic:=${IMAGE_TOPIC}"
    -p "camera_info_topic:=${CAMERA_INFO_TOPIC}"
    -p "image_output_topic:=${IMAGE_TOPIC}"
    -p "camera_info_output_topic:=${CAMERA_INFO_TOPIC}"
    -p "publish_image_stream:=true"
    -p "camera_frame_id:=${CAMERA_FRAME_ID}"
    -p "dictionary:=${TAG_DICTIONARY}"
    -p "tag_size_m:=${TAG_SIZE_M}"
    -p "target_tag_id:=${TARGET_TAG_ID}"
    -p "min_tag_area_px:=${MIN_TAG_AREA_PX}"
    -p "tag_pose_topic:=${TAG_POSE_TOPIC}")

  if [[ "${source}" == "device" ]]; then
    cmd+=(
      -p "video_device:=${VIDEO_DEVICE}"
      -p "device_width:=${DETECTOR_DEVICE_WIDTH}"
      -p "device_height:=${DETECTOR_DEVICE_HEIGHT}"
      -p "device_fps:=${DETECTOR_DEVICE_FPS}"
    )
  fi

  "${cmd[@]}" >"${DETECTOR_LOG}" 2>&1 &
  add_process "$!" "apriltag_detector"
}

start_landing_target() {
  echo "[run] apriltag_precision_landing_node -> ${LANDING_LOG}"
  ros2 run apriltag_precision_landing apriltag_precision_landing_node --ros-args \
    -p input_mode:=camera_pose \
    -p camera_tag_pose_topic:="${TAG_POSE_TOPIC}" \
    -p relay_image_stream:=true \
    -p image_input_topic:="${IMAGE_TOPIC}" \
    -p image_output_topic:=/image_raw \
    -p drone_pose_topic:="${DRONE_POSE_TOPIC}" \
    -p landing_target_topic:="${LANDING_TARGET_TOPIC}" \
    -p world_frame:="${WORLD_FRAME}" \
    -p input_timeout_sec:="${INPUT_TIMEOUT_SEC}" \
    -p camera_offset_x:="${CAMERA_OFFSET_X}" \
    -p camera_offset_y:="${CAMERA_OFFSET_Y}" \
    -p camera_offset_z:="${CAMERA_OFFSET_Z}" \
    -p camera_roll:="${CAMERA_ROLL}" \
    -p camera_pitch:="${CAMERA_PITCH}" \
    -p camera_yaw:="${CAMERA_YAW}" \
    -p publish_debug_tf:="${PUBLISH_DEBUG_TF}" \
    -p debug_tag_child_frame:="${DEBUG_TAG_CHILD_FRAME}" >"${LANDING_LOG}" 2>&1 &
  add_process "$!" "apriltag_landing_target"
}

start_image_view() {
  echo "[run] rqt_image_view (${IMAGE_TOPIC}) -> ${IMAGE_VIEW_LOG}"
  ros2 run rqt_image_view rqt_image_view "${IMAGE_TOPIC}" >"${IMAGE_VIEW_LOG}" 2>&1 &
  add_process "$!" "rqt_image_view"
}

main() {
  require_cmd ros2

  if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
  fi

  set +u
  source "${ROS_SETUP}"
  set -u

  if ! ros2 pkg prefix apriltag_precision_landing >/dev/null 2>&1; then
    echo "[error] apriltag_precision_landing package not found in overlay." >&2
    exit 1
  fi

  if is_true "${START_MAVROS}" && ! ros2 pkg prefix mavros >/dev/null 2>&1; then
    echo "[warn] mavros package not found in overlay; continuing without starting MAVROS." >&2
    START_MAVROS="0"
  fi

  if is_true "${START_CAMERA}"; then
    echo "[warn] START_CAMERA=1 requested, but this pipeline is configured to avoid v4l2. Forcing START_CAMERA=0." >&2
    START_CAMERA="0"
  fi

  if is_true "${START_IMAGE_VIEW}" && ! ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
    echo "[warn] rqt_image_view package not found; continuing without image viewer." >&2
    START_IMAGE_VIEW="0"
  fi

  if [[ "${KILL_BEFORE_LAUNCH}" == "1" ]]; then
    echo "[prep] stopping stale apriltag pipeline processes (if any)"
    kill_existing_stack
    sleep 1
  fi

  trap cleanup EXIT INT TERM

  if is_true "${START_MAVROS}"; then
    start_mavros
  fi

  if is_true "${START_CAMERA}"; then
    start_camera
  fi

  start_detector
  start_landing_target

  if is_true "${START_IMAGE_VIEW}"; then
    start_image_view
  fi

  echo "[ok] apriltag precision-landing pipeline started"
  echo "[info] startup mode: non-blocking best-effort (no topic wait gates)"
  echo "[info] START_MAVROS=${START_MAVROS} START_CAMERA=${START_CAMERA} START_IMAGE_VIEW=${START_IMAGE_VIEW}"
  echo "[info] detector_input_source=${DETECTOR_INPUT_SOURCE}"
  echo "[info] camera: device=${VIDEO_DEVICE} image=${IMAGE_TOPIC} info=${CAMERA_INFO_TOPIC}"
  echo "[info] tag: dict=${TAG_DICTIONARY} tag_size_m=${TAG_SIZE_M} target_tag_id=${TARGET_TAG_ID}"
  echo "[info] topics: tag_pose=${TAG_POSE_TOPIC} landing_target=${LANDING_TARGET_TOPIC} drone_pose=${DRONE_POSE_TOPIC}"
  echo "[info] logs:"
  if is_true "${START_MAVROS}"; then
    echo "  - ${MAVROS_LOG}"
  fi
  if is_true "${START_CAMERA}"; then
    echo "  - ${CAMERA_LOG}"
  fi
  echo "  - ${DETECTOR_LOG}"
  echo "  - ${LANDING_LOG}"
  if is_true "${START_IMAGE_VIEW}"; then
    echo "  - ${IMAGE_VIEW_LOG}"
  fi

  set +e
  wait -n "${PIDS[@]}"
  exit_code=$?
  set -e
  echo "[warn] one process exited (code=${exit_code}), stopping the others."
  exit "${exit_code}"
}

main "$@"
