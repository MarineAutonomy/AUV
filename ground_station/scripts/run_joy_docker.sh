#!/usr/bin/env bash
# Start joy teleop inside the long-lived auv_gs container (created at setup).
set -euo pipefail

GS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${AUV_GS_NAME:-auv_gs}"
DEV="${1:-/dev/input/js0}"

docker_cmd() {
  if docker info >/dev/null 2>&1; then
    docker "$@"
  elif sudo docker info >/dev/null 2>&1; then
    sudo docker "$@"
  else
    echo "ERROR: docker not usable" >&2
    exit 1
  fi
}

# Make sure idle container exists (no-op if already running).
bash "${GS_DIR}/scripts/ensure_gs_container.sh"

# Stop any previous joy launch inside the container.
docker_cmd exec "$NAME" bash -lc \
  'pkill -f "joy_teleop.launch.py|joy_thruster_teleop|joy_node" 2>/dev/null || true' \
  >/dev/null 2>&1 || true
sleep 0.3

echo "Starting joy launch in container '$NAME' (dev=$DEV)…"
# Idle container bypasses ENTRYPOINT; source ROS here. set +u — ROS setup.bash needs it.
# Do NOT `exec docker_cmd` — docker_cmd is a shell function (exec → exit 127).
docker_cmd exec -i "$NAME" bash -lc \
  "set +u && source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
   export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///gs/cyclonedds.xml && \
   exec ros2 launch auv_joy_teleop joy_teleop.launch.py dev:=${DEV}"
