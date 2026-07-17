#!/usr/bin/env bash

set -euo pipefail

MAVROS_PARAM_NODE="${MAVROS_PARAM_NODE:-}"
LEGACY_PARAM_GET_SERVICE="${LEGACY_PARAM_GET_SERVICE:-/mavros/param/get}"
SERVICE_TIMEOUT_SEC="${SERVICE_TIMEOUT_SEC:-10}"

PARAMETERS=(
  EKF2_EV_CTRL
  EKF2_EV_DELAY
  EKF2_EV_NOISE_MD
  EKF2_EVP_NOISE
  EKF2_EVA_NOISE
  EKF2_EV_POS_X
  EKF2_EV_POS_Y
  EKF2_EV_POS_Z
  EKF2_MAG_TYPE
  EKF2_OF_CTRL
  EKF2_RNG_CTRL
  EKF2_HGT_REF
  MAV_0_FORWARD
  MAV_0_RATE
)

timestamp() {
  date '+%Y-%m-%dT%H:%M:%S%z'
}

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'ERROR: ros2 is not available\n' >&2
  exit 1
fi

discover_modern_param_node() {
  local start_ts service_line service_name
  start_ts="$(date +%s)"
  while true; do
    service_line="$(ros2 service list -t 2>/dev/null |
      awk '$1 ~ /\/param\/get_parameters$/ && $0 ~ /rcl_interfaces\/srv\/GetParameters/ {print; exit}')"
    if [[ -n "${service_line}" ]]; then
      service_name="${service_line%% *}"
      MAVROS_PARAM_NODE="${service_name%/get_parameters}"
      return 0
    fi
    if (( $(date +%s) - start_ts >= SERVICE_TIMEOUT_SEC )); then
      return 1
    fi
    sleep 1
  done
}

query_modern_parameter() {
  local parameter="$1"
  timeout "${SERVICE_TIMEOUT_SEC}" ros2 param get \
    --timeout "${SERVICE_TIMEOUT_SEC}" "${MAVROS_PARAM_NODE}" "${parameter}" 2>&1
}

query_legacy_parameter() {
  local parameter="$1"
  timeout "${SERVICE_TIMEOUT_SEC}" ros2 service call \
    "${LEGACY_PARAM_GET_SERVICE}" mavros_msgs/srv/ParamGet \
    "{param_id: '${parameter}'}" 2>&1
}

interface=""
if [[ -n "${MAVROS_PARAM_NODE}" ]]; then
  if timeout "${SERVICE_TIMEOUT_SEC}" ros2 service type \
    "${MAVROS_PARAM_NODE}/get_parameters" 2>/dev/null |
    grep -Fx 'rcl_interfaces/srv/GetParameters' >/dev/null; then
    interface="modern"
  fi
elif discover_modern_param_node; then
  interface="modern"
fi

if [[ -z "${interface}" ]] && timeout "${SERVICE_TIMEOUT_SEC}" ros2 service type \
  "${LEGACY_PARAM_GET_SERVICE}" 2>/dev/null |
  grep -Fx 'mavros_msgs/srv/ParamGet' >/dev/null; then
  interface="legacy"
fi

if [[ -z "${interface}" ]]; then
  printf 'ERROR: no MAVROS parameter read interface found.\n' >&2
  printf 'Expected modern service: */param/get_parameters [rcl_interfaces/srv/GetParameters]\n' >&2
  printf 'Available MAVROS parameter services:\n' >&2
  ros2 service list -t 2>/dev/null | grep '/param/' >&2 || true
  exit 1
fi

printf 'PX4 LiDAR EKF parameter inspection\n'
printf 'timestamp=%s\n' "$(timestamp)"
if [[ "${interface}" == "modern" ]]; then
  printf 'interface=ROS 2 parameters\n'
  printf 'node=%s\n' "${MAVROS_PARAM_NODE}"
  printf 'service=%s/get_parameters\n' "${MAVROS_PARAM_NODE}"
else
  printf 'interface=legacy ParamGet\n'
  printf 'service=%s\n' "${LEGACY_PARAM_GET_SERVICE}"
fi
printf 'mode=read-only (this script never calls a parameter-set service)\n'
printf 'firmware_metadata_EKF2_EV_NOISE_MD_0=EV reported variance (parameter lower bound)\n'
printf 'required_EKF2_EV_CTRL=9 (horizontal position + yaw; no vertical position or velocity)\n\n'

failures=0
for parameter in "${PARAMETERS[@]}"; do
  if [[ "${interface}" == "modern" ]]; then
    response="$(query_modern_parameter "${parameter}" || true)"
    if grep -Eq '^(Integer|Double) value is:' <<<"${response}"; then
      value="$(sed -n 's/^\(Integer\|Double\) value is: /\1=/p' <<<"${response}" | tail -n1)"
      printf '%-18s %s\n' "${parameter}" "${value}"
    else
      printf '%-18s ERROR unavailable (%s)\n' "${parameter}" \
        "$(tr '\n' ' ' <<<"${response}" | sed 's/[[:space:]]\+/ /g')"
      failures=$((failures + 1))
    fi
  else
    response="$(query_legacy_parameter "${parameter}" || true)"
    if grep -q 'success=True' <<<"${response}"; then
      values="$(sed -n 's/.*integer=\([^,)]*\), real=\([^,)]*\).*/integer=\1 real=\2/p' \
        <<<"${response}" | tail -n1)"
      printf '%-18s %s\n' "${parameter}" "${values:-response_parse_failed}"
    else
      printf '%-18s ERROR unavailable\n' "${parameter}"
      failures=$((failures + 1))
    fi
  fi
done

printf '\nInterpretation checks (verify using the values above):\n'
printf '  EKF2_EV_CTRL must be 9.\n'
printf '  EKF2_EV_NOISE_MD must be 0 for message covariance with parameter lower bounds.\n'
printf '  EKF2_EV_POS_X/Y/Z should be 0 because the bridge publishes the compensated body pose.\n'
printf '  EKF2_EV_DELAY starts at 0 ms and requires later innovation-based tuning.\n'
printf '  Optical-flow and range aiding must remain enabled; range must remain the height reference.\n'
printf '  EKF2_MAG_TYPE is reported only and is never changed by this script.\n'
printf '  MAV_0_FORWARD should be 0 on the QGC/radio link before enabling RF2O odometry.\n'
printf '  Otherwise MAVLink ODOMETRY can be forwarded onto the low-bandwidth QGC link.\n'
printf '  Some builds expose only MAV_0_* and MAV_1_*; MAV_2_FORWARD is not required.\n'
printf '  PX4 must be rebooted after manually changing EKF2 parameters.\n'

if (( failures > 0 )); then
  printf '\nERROR: %d parameter(s) could not be read.\n' "${failures}" >&2
  exit 1
fi
