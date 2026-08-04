#!/usr/bin/env bash
# Read-only onboard readiness report. It sends no ROS command or PX4 request.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
SAMPLE_SEC="${SAMPLE_SEC:-5}"
HORIZONTAL_SCAN_TOPIC="${HORIZONTAL_SCAN_TOPIC:-/scan_slam}"
VERTICAL_SCAN_TOPIC="${VERTICAL_SCAN_TOPIC:-/scan_vertical}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[FAIL] ROS setup missing: ${ROS_SETUP}" >&2
  exit 2
fi

set +u
source "${ROS_SETUP}"
set -u

failures=0
warnings=0

pass() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; warnings=$((warnings + 1)); }
fail() { printf '[FAIL] %s\n' "$*"; failures=$((failures + 1)); }

topic_has_message() {
  local topic="$1"
  timeout "${SAMPLE_SEC}" ros2 topic echo "${topic}" --once >/dev/null 2>&1
}

publisher_count() {
  local topic="$1"
  local output
  output="$(ros2 topic info "${topic}" -v 2>/dev/null || true)"
  awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}' <<<"${output}"
}

printf 'Obstacle-avoidance readiness report\n'
printf 'workspace=%s\n' "${ROS_WS}"
printf 'branch=%s commit=%s\n' \
  "$(git -C "${ROS_WS}" branch --show-current 2>/dev/null || echo unknown)" \
  "$(git -C "${ROS_WS}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
printf 'load=%s\n' "$(cut -d' ' -f1-3 /proc/loadavg)"
if command -v vcgencmd >/dev/null 2>&1; then
  printf 'pi_throttled=%s\n' "$(vcgencmd get_throttled 2>/dev/null || echo unavailable)"
fi

declare -a required_topics=(
  /mavros/local_position/odom
  "${HORIZONTAL_SCAN_TOPIC}"
  "${VERTICAL_SCAN_TOPIC}"
  /mapping/local_obstacle_cloud
  /mapping/spatial_awareness/status
)
for topic in "${required_topics[@]}"; do
  if topic_has_message "${topic}"; then
    pass "message received on ${topic}"
  else
    fail "no message on ${topic} within ${SAMPLE_SEC}s"
  fi
done

if topic_has_message /mapping/global_cloud; then
  pass "global room cloud is publishing"
else
  warn "global room cloud is not publishing"
fi

printf '\n-- spatial awareness --\n'
spatial_status="$(timeout "${SAMPLE_SEC}" ros2 topic echo \
  /mapping/spatial_awareness/status --once 2>/dev/null || true)"
if [[ -n "${spatial_status}" ]]; then
  grep -A1 -E \
    'horizontal_sensor_state|vertical_sensor_state|odom_state|output_tf_state|local_cloud_update_rate_hz|local_point_count|front_state|rear_state|left_state|right_state|top_state|bottom_state|stale_sensor_warnings|tf_lookup_failures|processing_duration_ms' \
    <<<"${spatial_status}" || true
fi

printf '\n-- command path (inspection only) --\n'
raw_publishers="$(publisher_count /planner_cmd_vel_raw)"
guarded_publishers="$(publisher_count /planner_cmd_vel)"
mavros_publishers="$(publisher_count /mavros/setpoint_velocity/cmd_vel)"
printf '/planner_cmd_vel_raw publishers=%s\n' "${raw_publishers}"
printf '/planner_cmd_vel publishers=%s\n' "${guarded_publishers}"
printf '/mavros/setpoint_velocity/cmd_vel publishers=%s\n' "${mavros_publishers}"
if (( raw_publishers > 0 )) && (( guarded_publishers == 0 )); then
  fail "raw planner command exists without a guarded output publisher"
fi
if (( mavros_publishers > 0 )); then
  warn "a node currently publishes the MAVROS velocity setpoint topic"
fi

guard_status="$(timeout "${SAMPLE_SEC}" ros2 topic echo \
  /mapping/obstacle_avoidance/guard_status --once 2>/dev/null || true)"
if [[ -n "${guard_status}" ]]; then
  grep -A1 -E \
    'ready|guard_status|command_age_sec|odom_age_sec|awareness_age_sec|processing_duration_ms' \
    <<<"${guard_status}" || true
else
  warn "spatial command guard is not running (expected during mapping-only operation)"
fi

printf '\nsummary: failures=%d warnings=%d\n' "${failures}" "${warnings}"
if (( failures > 0 )); then
  exit 1
fi
