#!/usr/bin/env bash
set -euo pipefail

# Enables a non-root user in plugdev to boot Luxonis OAK / Movidius devices.
RULE_SOURCE="${HOME}/.codex/80-depthai-movidius.rules"
RULE_DESTINATION="/etc/udev/rules.d/80-depthai-movidius.rules"

if [[ ! -f "${RULE_SOURCE}" ]]; then
    echo "Missing rule file: ${RULE_SOURCE}" >&2
    exit 1
fi

sudo install -m 0644 "${RULE_SOURCE}" "${RULE_DESTINATION}"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=03e7
sudo udevadm settle
echo "DepthAI udev rule installed. Disconnect/reconnect the OAK once if it is still not enumerated."
