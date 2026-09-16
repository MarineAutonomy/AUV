docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile_timi \
  -t aatmaj9/timi:4.0 \
  --push \
  .