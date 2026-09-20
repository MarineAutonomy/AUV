#!/bin/sh
# Wrap image entrypoint so SonarLink only discovers Ping2 + Ping360 USB serial.
set -e
FILTER="/opt/mavlab/sonarlink-serial-filter.cjs"
if [ -f "$FILTER" ]; then
  export NODE_OPTIONS="${NODE_OPTIONS:+$NODE_OPTIONS }--require=$FILTER"
  echo "[sonarview] serial discovery filter enabled (Ping2/Ping360 only)"
fi
exec /usr/local/bin/entrypoint.sh
