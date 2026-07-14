#!/usr/bin/env bash

set -euo pipefail

PARAM_GET_SERVICE="${PARAM_GET_SERVICE:-/mavros/param/get}"
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
)

timestamp() {
  date '+%Y-%m-%dT%H:%M:%S%z'
}

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'ERROR: ros2 is not available\n' >&2
  exit 1
fi

if ! timeout "${SERVICE_TIMEOUT_SEC}" ros2 service type "${PARAM_GET_SERVICE}" 2>/dev/null |
  grep -Fx 'mavros_msgs/srv/ParamGet' >/dev/null; then
  printf 'ERROR: MAVROS parameter service unavailable: %s\n' "${PARAM_GET_SERVICE}" >&2
  exit 1
fi

printf 'PX4 LiDAR EKF parameter inspection\n'
printf 'timestamp=%s\n' "$(timestamp)"
printf 'service=%s\n' "${PARAM_GET_SERVICE}"
printf 'mode=read-only (this script never calls a parameter-set service)\n'
printf 'firmware_metadata_EKF2_EV_NOISE_MD_0=EV reported variance (parameter lower bound)\n'
printf 'required_EKF2_EV_CTRL=9 (horizontal position + yaw; no vertical position or velocity)\n\n'

failures=0
for parameter in "${PARAMETERS[@]}"; do
  response="$(timeout "${SERVICE_TIMEOUT_SEC}" ros2 service call \
    "${PARAM_GET_SERVICE}" mavros_msgs/srv/ParamGet "{param_id: '${parameter}'}" 2>&1 || true)"
  if grep -q 'success=True' <<<"${response}"; then
    values="$(sed -n 's/.*integer=\([^,)]*\), real=\([^,)]*\).*/integer=\1 real=\2/p' <<<"${response}" | tail -n1)"
    printf '%-18s %s\n' "${parameter}" "${values:-response_parse_failed}"
  else
    printf '%-18s ERROR unavailable\n' "${parameter}"
    failures=$((failures + 1))
  fi
done

printf '\nInterpretation checks (verify using the values above):\n'
printf '  EKF2_EV_CTRL must be 9.\n'
printf '  EKF2_EV_NOISE_MD must be 0 for message covariance with parameter lower bounds.\n'
printf '  EKF2_EV_POS_X/Y/Z should be 0 because the bridge publishes the compensated body pose.\n'
printf '  EKF2_EV_DELAY starts at 0 ms and requires later innovation-based tuning.\n'
printf '  Optical-flow and range aiding must remain enabled; range must remain the height reference.\n'
printf '  EKF2_MAG_TYPE is reported only and is never changed by this script.\n'
printf '  PX4 must be rebooted after manually changing EKF2 parameters.\n'

if (( failures > 0 )); then
  printf '\nERROR: %d parameter(s) could not be read.\n' "${failures}" >&2
  exit 1
fi
