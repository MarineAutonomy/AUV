#!/usr/bin/env bash
# Stop joy nodes inside auv_gs; leave the idle container running.
set -euo pipefail

NAME="${AUV_GS_NAME:-auv_gs}"

docker_cmd() {
  if docker info >/dev/null 2>&1; then
    docker "$@"
  elif sudo docker info >/dev/null 2>&1; then
    sudo docker "$@"
  else
    exit 0
  fi
}

if ! docker_cmd ps --format '{{.Names}}' | grep -qx "$NAME"; then
  # Legacy one-shot container
  docker_cmd rm -f "${AUV_GS_JOY_NAME:-auv_gs_joy}" >/dev/null 2>&1 || true
  exit 0
fi

docker_cmd exec "$NAME" bash -lc \
  'pkill -f "joy_teleop.launch.py|joy_thruster_teleop|joy_node|ros2 launch auv_joy_teleop" 2>/dev/null || true' \
  >/dev/null 2>&1 || true

echo "Stopped joy inside '$NAME' (container left running)"
