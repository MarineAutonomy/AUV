#!/usr/bin/env bash
# Ensure the long-lived ground-station container is running (started at setup).
# Joy Connect then only docker-execs ros2 launch into this container.
set -euo pipefail

GS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${AUV_GS_IMAGE:-auv-ground-station:latest}"
NAME="${AUV_GS_NAME:-auv_gs}"
CYCLONE_XML="${GS_DIR}/cyclonedds.xml"

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

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not found. Run: ./setup_env.sh --docker" >&2
  exit 1
fi
if ! docker_cmd image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "ERROR: image '$IMAGE' missing. Run: ./setup_env.sh --docker" >&2
  exit 1
fi
if [ ! -f "$CYCLONE_XML" ]; then
  echo "ERROR: missing $CYCLONE_XML" >&2
  exit 1
fi

# Already up?
if docker_cmd ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "ground-station container '$NAME' already running"
  exit 0
fi

# Remove stale stopped container with same name
docker_cmd rm -f "$NAME" >/dev/null 2>&1 || true
# Legacy one-shot name from older builds
docker_cmd rm -f "${AUV_GS_JOY_NAME:-auv_gs_joy}" >/dev/null 2>&1 || true

extra_args=()
if [ -d /dev/input ]; then
  extra_args+=(-v /dev/input:/dev/input)
fi
# Hot-plug joysticks: privileged helps /dev nodes appear inside the container
extra_args+=(--privileged)

echo "Starting ground-station container '$NAME' (idle)…"
# Bypass image ENTRYPOINT for idle sleep — entrypoint sources ROS and used to crash with set -u.
docker_cmd run -d \
  --name "$NAME" \
  --restart unless-stopped \
  --network host \
  --ipc host \
  --entrypoint sleep \
  "${extra_args[@]}" \
  -v "${CYCLONE_XML}:/gs/cyclonedds.xml:ro" \
  -v /tmp:/tmp \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///gs/cyclonedds.xml \
  "$IMAGE" \
  infinity

echo "ground-station container '$NAME' is up (Connect will start joy inside it)"
