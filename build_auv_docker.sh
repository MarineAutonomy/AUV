docker buildx build \
  --platform linux/arm64 \
  --pull \
  -f .devcontainer/Dockerfile_timi \
  -t aatmaj9/timi:3.0 \
  --push \
  .