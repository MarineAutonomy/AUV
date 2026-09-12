#!/usr/bin/env bash
# Ensure the long-lived ground-station container is running (started at setup).
# MAV-GUI docker-execs the PC acoustic modem (role a) into this container.
set -euo pipefail

GS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="${AUV_GS_IMAGE_REPO:-aatmaj9/auv-ground-station}"
MAV_GUI_DATA_DIR="${MAV_GUI_DATA_DIR:-${HOME}/.local/share/mav-gui}"
VERSION_FILE="${MAV_GUI_DATA_DIR}/gs_image_version"

if [ -n "${AUV_GS_IMAGE:-}" ]; then
  IMAGE="$AUV_GS_IMAGE"
elif [ -n "${AUV_GS_IMAGE_VERSION:-}" ]; then
  IMAGE="${REPO}:${AUV_GS_IMAGE_VERSION}"
elif [ -f "$VERSION_FILE" ]; then
  IMAGE="${REPO}:$(tr -d '[:space:]' < "$VERSION_FILE")"
else
  IMAGE="${REPO}:1.0"
fi

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
  echo "image '$IMAGE' missing locally — pulling…"
  if ! docker_cmd pull "$IMAGE"; then
    echo "ERROR: pull failed. Run: ./setup_env.sh --docker  (or push with ./build_gs_docker.sh <version>)" >&2
    exit 1
  fi
fi
if [ ! -f "$CYCLONE_XML" ]; then
  echo "ERROR: missing $CYCLONE_XML" >&2
  exit 1
fi

WANT_ID="$(docker_cmd image inspect -f '{{.Id}}' "$IMAGE")"

# If already running, only keep it when it is from the target image (not an older tag).
if docker_cmd ps --format '{{.Names}}' | grep -qx "$NAME"; then
  HAVE_ID="$(docker_cmd inspect -f '{{.Image}}' "$NAME" 2>/dev/null || true)"
  if [ -n "$HAVE_ID" ] && [ "$HAVE_ID" = "$WANT_ID" ]; then
    echo "ground-station container '$NAME' already running from $IMAGE"
    exit 0
  fi
  echo "container '$NAME' is running from a different image — recreating from $IMAGE…"
  docker_cmd rm -f "$NAME" >/dev/null 2>&1 || true
fi

# Remove stale stopped container with same name
docker_cmd rm -f "$NAME" >/dev/null 2>&1 || true
# Drop leftover joy-era container name
docker_cmd rm -f "${AUV_GS_JOY_NAME:-auv_gs_joy}" >/dev/null 2>&1 || true

extra_args=()
# USB acoustic modem (and any other /dev nodes)
extra_args+=(-v /dev:/dev)
extra_args+=(--privileged)

echo "Starting ground-station container '$NAME' (idle) from $IMAGE…"
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

echo "ground-station container '$NAME' is up (Modem tab starts role a inside it)"
