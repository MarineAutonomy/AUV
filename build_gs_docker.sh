#!/usr/bin/env bash
# Build & push a versioned ground-station image to Docker Hub (amd64 operator PCs).
# Requires: docker login, buildx.
#
# Usage:
#   ./build_gs_docker.sh 1.0
#   ./build_gs_docker.sh 2.0
#   AUV_GS_IMAGE_REPO=myuser/auv-ground-station ./build_gs_docker.sh 2.1
#
# GUI / setup pull version tags (1.0, 2.0, …) — not a floating "latest" tag.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${AUV_GS_IMAGE_REPO:-aatmaj9/auv-ground-station}"
VERSION="${1:-}"

if [[ ! "$VERSION" =~ ^[0-9]+(\.[0-9]+)*$ ]]; then
  echo "Usage: $0 <version>   e.g. $0 1.0" >&2
  echo "Version must look like 1.0 or 2.1.3 (digits and dots only)." >&2
  exit 1
fi

IMAGE="${REPO}:${VERSION}"

docker buildx build \
  --platform linux/amd64 \
  --pull \
  --build-arg "GS_IMAGE_VERSION=${VERSION}" \
  -f "${ROOT}/ground_station/Dockerfile" \
  -t "${IMAGE}" \
  --push \
  "${ROOT}/ground_station"

echo "Pushed ${IMAGE}"
echo "Operators on older builds will be offered ${VERSION} on next GUI launch."
