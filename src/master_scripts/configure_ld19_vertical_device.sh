#!/usr/bin/env bash
# Create a stable, non-world-writable device name for the vertical LD19.

set -euo pipefail

DEVICE="${1:-}"
SYMLINK_NAME="${2:-ldlidar_vertical}"
RULE_FILE="/etc/udev/rules.d/99-ld19-vertical.rules"

if [[ -z "${DEVICE}" ]]; then
  echo "Usage: sudo $0 /dev/ttyUSB<N> [symlink-name]" >&2
  exit 2
fi
if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this setup once with sudo." >&2
  exit 2
fi
if [[ ! "${SYMLINK_NAME}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Invalid symlink name: ${SYMLINK_NAME}" >&2
  exit 2
fi

DEVICE="$(readlink -f "${DEVICE}")"
if [[ ! -c "${DEVICE}" ]]; then
  echo "Not a character device: ${DEVICE}" >&2
  exit 2
fi
command -v udevadm >/dev/null 2>&1 || {
  echo "udevadm is required" >&2
  exit 2
}

properties="$(udevadm info --query=property --name="${DEVICE}")"
vendor="$(awk -F= '$1 == "ID_VENDOR_ID" {print $2; exit}' <<<"${properties}")"
product="$(awk -F= '$1 == "ID_MODEL_ID" {print $2; exit}' <<<"${properties}")"
serial="$(awk -F= '$1 == "ID_SERIAL_SHORT" {print $2; exit}' <<<"${properties}")"
device_path="$(awk -F= '$1 == "ID_PATH" {print $2; exit}' <<<"${properties}")"

if [[ -z "${vendor}" || -z "${product}" ]]; then
  echo "Could not read USB vendor/product identity for ${DEVICE}" >&2
  exit 1
fi

if [[ -n "${serial}" ]]; then
  identity_rule="ATTRS{serial}==\"${serial}\""
  identity_summary="serial=${serial}"
elif [[ -n "${device_path}" ]]; then
  identity_rule="ENV{ID_PATH}==\"${device_path}\""
  identity_summary="USB path=${device_path}"
else
  echo "The adapter has neither a serial number nor a stable USB path." >&2
  echo "Do not create a VID/PID-only rule when another serial LiDAR is installed." >&2
  exit 1
fi

printf '%s\n' \
  "# Vertical LD19 selected from ${DEVICE}; generated $(date --iso-8601=seconds)" \
  "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${vendor}\", ATTRS{idProduct}==\"${product}\", ${identity_rule}, GROUP=\"dialout\", MODE=\"0660\", TAG+=\"uaccess\", SYMLINK+=\"${SYMLINK_NAME}\"" \
  >"${RULE_FILE}"

if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
  usermod -aG dialout "${SUDO_USER}"
fi

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
udevadm settle

echo "Installed ${RULE_FILE} using ${identity_summary}."
echo "Expected device: /dev/${SYMLINK_NAME}"
echo "Unplug/replug the adapter. Log out and back in if dialout membership was added."
