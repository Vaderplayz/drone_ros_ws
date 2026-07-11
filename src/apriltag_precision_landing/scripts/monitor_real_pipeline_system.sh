#!/usr/bin/env bash

set -euo pipefail

INTERVAL_SEC="${INTERVAL_SEC:-1}"

while true; do
  timestamp="$(date --iso-8601=ns)"
  load="$(cut -d' ' -f1-3 /proc/loadavg 2>/dev/null || printf 'unavailable')"
  mem_available_kb="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || true)"
  temperature_c="unavailable"
  throttled="unavailable"

  if [[ -r /sys/class/thermal/thermal_zone0/temp ]]; then
    raw_temp="$(< /sys/class/thermal/thermal_zone0/temp)"
    temperature_c="$(awk -v value="${raw_temp}" 'BEGIN {printf "%.1f", value / 1000.0}')"
  fi

  if command -v vcgencmd >/dev/null 2>&1; then
    throttled="$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2 || printf 'unavailable')"
  fi

  process_stats="$(ps -eo comm=,pid=,%cpu=,rss=,args= | \
    awk '/apriltag_camera_detector_node|apriltag_precision_landing_node|mavros_node/ && !/awk/ {printf "%s:%s:cpu=%s:rss_kb=%s;", $1, $2, $3, $4}')"

  printf 'SYSTEM_DIAG timestamp=%s load=%s mem_available_kb=%s cpu_temp_c=%s throttled=%s processes=%s\n' \
    "${timestamp}" "${load}" "${mem_available_kb:-unavailable}" "${temperature_c}" "${throttled}" \
    "${process_stats:-none}"

  sleep "${INTERVAL_SEC}"
done
