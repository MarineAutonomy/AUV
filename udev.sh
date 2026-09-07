#!/bin/bash
# Apply USB serial udev rules. Prefer already-root (GUI wraps with sudo -S).
# Use non-interactive sudo (-n) for CLI so missing credentials fail fast
# instead of hanging on a TTY password prompt over SSH.
set -euo pipefail

run() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo -n "$@"
  fi
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/99-usb-serial.rules"

if [ ! -f "$RULES_SRC" ]; then
  echo "error: missing rules file: $RULES_SRC" >&2
  exit 1
fi

run cp "$RULES_SRC" /etc/udev/rules.d/99-usb-serial.rules
run udevadm control --reload-rules
run udevadm trigger
echo "udev rules applied"
