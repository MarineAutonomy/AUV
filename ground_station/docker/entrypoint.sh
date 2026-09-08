#!/usr/bin/env bash
# Source ROS + workspace, then exec the command (default: joy launch).
# ROS setup.bash references optional unset vars — incompatible with `set -u`.
set -eo pipefail
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi
set -u
exec "$@"
