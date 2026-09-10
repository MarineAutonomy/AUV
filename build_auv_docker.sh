docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile_timi \
  -t aatmaj9/timi:2.0 \
  --push \
  .