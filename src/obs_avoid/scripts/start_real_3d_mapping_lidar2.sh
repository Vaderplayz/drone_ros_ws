#!/usr/bin/env bash
# Compatibility wrapper. Main real-drone startup scripts live in src/master_scripts.
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
exec "${SCRIPT_DIR}/../../master_scripts/start_real_3d_mapping_lidar2.sh" "$@"
