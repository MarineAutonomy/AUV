#!/usr/bin/env bash
# Ground station ROS / CycloneDDS setup.
#
# Default (new PC — Docker ROS, no host Humble install):
#   ./setup_env.sh
#   ./setup_env.sh --docker
#
# Native host ROS (old path):
#   ./setup_env.sh --native
#
# After that, every new terminal loads env via ~/.bashrc.
# Manual (any shell): source this file.

GS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QUIET="${GS_SETUP_QUIET:-0}"
CYCLONE_XML="${GS_DIR}/cyclonedds.xml"
MARKER="# >>> AUV ground station ROS env >>>"
DOCKER_IMAGE="${AUV_GS_IMAGE:-auv-ground-station:latest}"
USE_DOCKER_MARKER="${GS_DIR}/.use_docker"

# Jetson peers (edit here or in cyclonedds.xml)
JETSON_LAN_IP="${JETSON_LAN_IP:-192.168.194.10}"
JETSON_WIFI_IP="${JETSON_WIFI_IP:-192.168.0.50}"

log() {
  if [ "$QUIET" != "1" ]; then
    echo "$*"
  fi
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

# ROS setup.bash references optional unset vars — incompatible with `set -u`.
source_ros_file() {
  local file="$1"
  [ -f "$file" ] || return 1
  local had_u=0
  [[ $- == *u* ]] && had_u=1
  set +u
  # shellcheck disable=SC1091
  source "$file"
  [ "$had_u" -eq 1 ] && set -u
  return 0
}

detect_wifi_iface() {
  local iface=""
  if command -v iw >/dev/null 2>&1; then
    iface="$(iw dev 2>/dev/null | awk '$1 == "Interface" { print $2; exit }')"
  fi
  if [ -z "$iface" ]; then
    iface="$(ip -o link show 2>/dev/null | awk -F': ' '{print $2}' | grep -E '^(wl|wlan)' | head -n1 || true)"
  fi
  if [ -z "$iface" ]; then
    iface="$(ls /sys/class/net 2>/dev/null | grep -E '^(wl|wlan)' | head -n1 || true)"
  fi
  printf '%s' "$iface"
}

patch_cyclone_xml() {
  local iface="$1"
  [ -f "$CYCLONE_XML" ] || die "missing $CYCLONE_XML"

  # Rewrite Wi‑Fi NIC only — never overwrite loopback (lo) used for same-PC /joy.
  if command -v python3 >/dev/null 2>&1; then
    JETSON_LAN_IP="$JETSON_LAN_IP" JETSON_WIFI_IP="$JETSON_WIFI_IP" CYCLONE_XML="$CYCLONE_XML" WIFI_IFACE="$iface" python3 - <<'PY'
import os, re
path = os.environ["CYCLONE_XML"]
lan = os.environ["JETSON_LAN_IP"]
wifi = os.environ["JETSON_WIFI_IP"]
iface = os.environ["WIFI_IFACE"]
text = open(path, encoding="utf-8").read()

def repl_iface(m):
    name = m.group(1)
    if name == "lo" or not iface:
        return m.group(0)
    return f'<NetworkInterface name="{iface}"'

text = re.sub(r'<NetworkInterface name="([^"]*)"', repl_iface, text)

peers = f'''      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="{lan}"/>
        <Peer address="{wifi}"/>
      </Peers>'''
text2, n = re.subn(r"<Peers>.*?</Peers>", peers, text, count=1, flags=re.S)
if n:
    text = text2
open(path, "w", encoding="utf-8").write(text)
PY
  elif grep -q '<NetworkInterface name=' "$CYCLONE_XML"; then
    # Fallback: only replace non-lo interfaces
    sed -i -E "/NetworkInterface name=\"lo\"/!s|<NetworkInterface name=\"[^\"]*\"|<NetworkInterface name=\"${iface}\"|" "$CYCLONE_XML"
  fi
}

apply_env() {
  if ! source_ros_file /opt/ros/humble/setup.bash; then
    if [ "$QUIET" != "1" ]; then
      echo "ROS 2 Humble not found at /opt/ros/humble — run: ./setup_env.sh" >&2
    fi
  fi

  source_ros_file "$GS_DIR/ros2_ws/install/setup.bash" || true

  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://${CYCLONE_XML}"

  if [ "$QUIET" != "1" ]; then
    local iface
    iface="$(grep -oE 'NetworkInterface name="[^"]+"' "$CYCLONE_XML" 2>/dev/null | grep -v 'name="lo"' | head -1 | cut -d'"' -f2 || true)"
    echo "Ground station ROS env ready"
    echo "  domain=$ROS_DOMAIN_ID  rmw=$RMW_IMPLEMENTATION"
    echo "  iface=${iface:-?}  cyclone=$CYCLONEDDS_URI"
    echo "  peers: 127.0.0.1 (local)  $JETSON_LAN_IP (LAN)  $JETSON_WIFI_IP (Wi‑Fi)"
  fi
}

install_bashrc() {
  local bashrc="${HOME}/.bashrc"
  # Drop legacy PC/ block if present
  if [ -f "$bashrc" ] && grep -qF "# >>> AUV PC ROS env >>>" "$bashrc" 2>/dev/null; then
    sed -i '/# >>> AUV PC ROS env >>>/,/# <<< AUV PC ROS env <<</d' "$bashrc"
    log "bashrc: removed legacy PC/ auto-load"
  fi
  if [ -f "$bashrc" ] && grep -qF "$MARKER" "$bashrc" 2>/dev/null; then
    log "bashrc: already installed"
    return 0
  fi
  cat >> "$bashrc" <<EOF

$MARKER
# Auto-load ground station ROS / CycloneDDS. Quiet on every shell.
if [ -f "$GS_DIR/setup_env.sh" ]; then
  GS_SETUP_QUIET=1 source "$GS_DIR/setup_env.sh"
fi
# <<< AUV ground station ROS env <<<
EOF
  log "bashrc: added auto-load to $bashrc"
}

need_sudo() {
  if [ "$(id -u)" -eq 0 ]; then
    return 1
  fi
  command -v sudo >/dev/null 2>&1 || die "need root or sudo"
  return 0
}

apt_install_ros() {
  local -a pkgs=(
    ros-humble-desktop
    ros-humble-rmw-cyclonedds-cpp
    ros-humble-joy
    python3-colcon-common-extensions
  )
  local missing=()
  local p
  for p in "${pkgs[@]}"; do
    if ! dpkg -s "$p" >/dev/null 2>&1; then
      missing+=("$p")
    fi
  done
  if [ "${#missing[@]}" -eq 0 ]; then
    log "apt: ROS packages already installed"
    return 0
  fi

  log "apt: installing ${missing[*]}"
  if need_sudo; then
    sudo apt-get update
    sudo apt-get install -y "${missing[@]}"
  else
    apt-get update
    apt-get install -y "${missing[@]}"
  fi
}

# CycloneDDS XML requires SocketReceiveBufferSize min=10MB; default Ubuntu rmem_max is ~425KB.
install_sysctl_buffers() {
  local conf="/etc/sysctl.d/60-auv-ros2-buffers.conf"
  local need_write=0
  local rmem
  rmem="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
  if [ "${rmem:-0}" -lt 10485760 ]; then
    need_write=1
  fi
  if [ ! -f "$conf" ]; then
    need_write=1
  fi

  if [ "$need_write" -eq 0 ]; then
    log "sysctl: net.core.rmem_max=$rmem (OK)"
    return 0
  fi

  log "sysctl: raising UDP receive buffers for CycloneDDS (needs sudo)…"
  local body
  body="$(cat <<'EOF'
# AUV ground station — CycloneDDS (matches Jetson 60-auv-ros2-buffers.conf)
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
net.core.rmem_max=2147483647
EOF
)"
  if need_sudo; then
    echo "$body" | sudo tee "$conf" >/dev/null
    sudo sysctl -p "$conf" >/dev/null
  else
    echo "$body" > "$conf"
    sysctl -p "$conf" >/dev/null
  fi
  rmem="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
  log "sysctl: net.core.rmem_max=$rmem"
}

build_joy() {
  [ -f /opt/ros/humble/setup.bash ] || die "ROS Humble missing after apt install"
  source_ros_file /opt/ros/humble/setup.bash || die "failed to source ROS Humble"
  command -v colcon >/dev/null 2>&1 || die "colcon not found (python3-colcon-common-extensions)"

  mkdir -p "$GS_DIR/ros2_ws/src"
  cd "$GS_DIR/ros2_ws"
  log "colcon: building auv_joy_teleop…"
  colcon build --packages-select auv_joy_teleop
  source_ros_file "$GS_DIR/ros2_ws/install/setup.bash" || die "failed to source ros2_ws install"
  log "colcon: OK"
}

docker_cmd() {
  if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
    docker "$@"
  elif command -v sudo >/dev/null 2>&1 && sudo docker info >/dev/null 2>&1; then
    sudo docker "$@"
  else
    return 1
  fi
}

install_docker() {
  if command -v docker >/dev/null 2>&1 && docker_cmd info >/dev/null 2>&1; then
    log "docker: OK ($(docker --version 2>/dev/null | head -1))"
    return 0
  fi

  log "docker: installing docker.io…"
  if need_sudo; then
    sudo apt-get update
    sudo apt-get install -y docker.io
    sudo systemctl enable --now docker || true
    if [ -n "${SUDO_USER:-}" ]; then
      sudo usermod -aG docker "$SUDO_USER" || true
    elif [ -n "${USER:-}" ] && [ "$USER" != "root" ]; then
      sudo usermod -aG docker "$USER" || true
    fi
  else
    apt-get update
    apt-get install -y docker.io
    systemctl enable --now docker || true
  fi

  if ! docker_cmd info >/dev/null 2>&1; then
    die "docker installed but daemon not usable yet — log out/in (docker group) or: sudo systemctl start docker"
  fi
  log "docker: installed. If 'permission denied', log out/in so group 'docker' applies."
}

build_docker_image() {
  [ -f "$GS_DIR/Dockerfile" ] || die "missing $GS_DIR/Dockerfile"
  chmod +x "$GS_DIR/docker/entrypoint.sh" "$GS_DIR/scripts/run_joy_docker.sh" 2>/dev/null || true
  log "docker: building $DOCKER_IMAGE (several minutes on first run)…"
  if ! docker_cmd build -t "$DOCKER_IMAGE" -f "$GS_DIR/Dockerfile" "$GS_DIR"; then
    die "docker build failed"
  fi
  touch "$USE_DOCKER_MARKER"
  log "docker: image ready ($DOCKER_IMAGE)"
}

# Prefer pull when image exists on a registry; otherwise build locally.
ensure_docker_image() {
  install_docker
  chmod +x "$GS_DIR/docker/entrypoint.sh" \
    "$GS_DIR/scripts/run_joy_docker.sh" \
    "$GS_DIR/scripts/ensure_gs_container.sh" \
    "$GS_DIR/scripts/stop_joy_docker.sh" 2>/dev/null || true

  log "docker: trying pull $DOCKER_IMAGE …"
  if docker_cmd pull "$DOCKER_IMAGE"; then
    touch "$USE_DOCKER_MARKER"
    log "docker: pulled $DOCKER_IMAGE"
    return 0
  fi

  log "docker: pull failed or image not on registry — building locally"
  build_docker_image
}

using_docker() {
  [ -f "$USE_DOCKER_MARKER" ] || [ "${GS_JOY_DOCKER:-0}" = "1" ]
}

pass() { echo "  PASS  $*"; }
fail() { echo "  FAIL  $*"; FAILS=$((FAILS + 1)); }
warn() { echo "  WARN  $*"; WARNS=$((WARNS + 1)); }

run_self_test() {
  FAILS=0
  WARNS=0
  echo
  echo "=== Self-test ==="

  QUIET=1 apply_env

  if using_docker; then
    if command -v docker >/dev/null 2>&1 && docker_cmd info >/dev/null 2>&1; then
      pass "docker usable"
    else
      fail "docker not usable"
    fi
    if docker_cmd image inspect "$DOCKER_IMAGE" >/dev/null 2>&1; then
      pass "image $DOCKER_IMAGE present"
    else
      fail "image $DOCKER_IMAGE missing"
    fi
    if [ -x "$GS_DIR/scripts/run_joy_docker.sh" ]; then
      pass "scripts/run_joy_docker.sh"
    else
      fail "scripts/run_joy_docker.sh missing/not executable"
    fi
    if [ -x "$GS_DIR/scripts/ensure_gs_container.sh" ]; then
      pass "scripts/ensure_gs_container.sh"
    else
      fail "scripts/ensure_gs_container.sh missing/not executable"
    fi
    if docker_cmd ps --format '{{.Names}}' | grep -qx "${AUV_GS_NAME:-auv_gs}"; then
      pass "idle container ${AUV_GS_NAME:-auv_gs} running"
    else
      fail "idle container ${AUV_GS_NAME:-auv_gs} not running"
    fi
  else
    if command -v ros2 >/dev/null 2>&1; then
      pass "ros2 on PATH ($(command -v ros2))"
    else
      fail "ros2 not on PATH"
    fi
    if ros2 pkg prefix auv_joy_teleop >/dev/null 2>&1; then
      pass "package auv_joy_teleop found"
    else
      fail "package auv_joy_teleop not found"
    fi
    if ros2 pkg prefix joy >/dev/null 2>&1; then
      pass "package joy found"
    else
      fail "package joy not found"
    fi
  fi

  if [ "${ROS_DOMAIN_ID:-}" = "0" ]; then
    pass "ROS_DOMAIN_ID=0"
  else
    fail "ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-unset}' (want 0)"
  fi

  if [ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ]; then
    pass "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  else
    fail "RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-unset}'"
  fi

  if [ -f "$CYCLONE_XML" ] && [ "${CYCLONEDDS_URI:-}" = "file://${CYCLONE_XML}" ]; then
    pass "CYCLONEDDS_URI points at cyclonedds.xml"
  else
    fail "CYCLONEDDS_URI='${CYCLONEDDS_URI:-unset}'"
  fi

  local rmem
  rmem="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
  if [ "${rmem:-0}" -ge 10485760 ]; then
    pass "net.core.rmem_max=$rmem (>= 10MB for Cyclone)"
  else
    warn "net.core.rmem_max=$rmem (< 10MB) — run ./setup_env.sh (sudo) for full buffers"
  fi

  local iface
  iface="$(grep -oE 'NetworkInterface name="[^"]+"' "$CYCLONE_XML" 2>/dev/null | grep -v 'name="lo"' | head -1 | cut -d'"' -f2 || true)"
  if grep -q 'NetworkInterface name="lo"' "$CYCLONE_XML" 2>/dev/null; then
    warn "Cyclone XML lists lo — remove it (breaks multicast; use Peer 127.0.0.1 instead)"
  else
    pass "Cyclone has no lo interface (Wi‑Fi + Peer 127.0.0.1 for local)"
  fi
  if [ -n "$iface" ] && [ -d "/sys/class/net/$iface" ]; then
    pass "Cyclone NIC '$iface' exists"
    if [ -f "/sys/class/net/$iface/operstate" ]; then
      local state
      state="$(cat "/sys/class/net/$iface/operstate" 2>/dev/null || true)"
      if [ "$state" = "up" ]; then
        pass "NIC '$iface' is up"
      else
        warn "NIC '$iface' operstate=$state (connect Wi‑Fi if needed)"
      fi
    fi
  else
    fail "Cyclone NIC '${iface:-?}' not found on this machine"
  fi

  if grep -qF "$JETSON_LAN_IP" "$CYCLONE_XML" && grep -qF "$JETSON_WIFI_IP" "$CYCLONE_XML"; then
    pass "Peers in XML: $JETSON_LAN_IP , $JETSON_WIFI_IP"
  else
    fail "Peers missing from $CYCLONE_XML"
  fi

  if [ -f "${HOME}/.bashrc" ] && grep -qF "$MARKER" "${HOME}/.bashrc"; then
    pass "bashrc auto-load installed"
  else
    fail "bashrc auto-load missing"
  fi

  echo
  if [ "$FAILS" -gt 0 ]; then
    echo "Self-test: $FAILS failed, $WARNS warnings"
    return 1
  fi
  echo "Self-test: all required checks passed ($WARNS warnings)"
  return 0
}

print_done() {
  local iface
  iface="$(grep -oE 'NetworkInterface name="[^"]+"' "$CYCLONE_XML" 2>/dev/null | grep -v 'name="lo"' | head -1 | cut -d'"' -f2 || true)"
  if using_docker; then
    cat <<EOF

======== PC setup complete (Docker) ========
Cyclone NIC:  ${iface:-unknown}
Image:        $DOCKER_IMAGE
Container:    ${AUV_GS_NAME:-auv_gs} (idle — stays running)
Jetson LAN:   $JETSON_LAN_IP
Jetson Wi‑Fi: $JETSON_WIFI_IP

Joy (CLI):    $GS_DIR/scripts/run_joy_docker.sh
Or use MAV-GUI Xbox tab → Connect (starts joy inside the container)

Re-test:      ./setup_env.sh --test
Native ROS:   ./setup_env.sh --native
============================================
EOF
  else
    cat <<EOF

======== PC setup complete (native ROS) ========
Cyclone NIC:  ${iface:-unknown}
Jetson LAN:   $JETSON_LAN_IP
Jetson Wi‑Fi: $JETSON_WIFI_IP

  ros2 launch auv_joy_teleop joy_teleop.launch.py

Re-test:      ./setup_env.sh --test
Docker mode:  ./setup_env.sh --docker
================================================
EOF
  fi
}

bootstrap_docker() {
  set -euo pipefail
  echo "=== AUV ground station setup (Docker) ==="
  echo "GS_DIR=$GS_DIR"
  echo

  local iface
  iface="$(detect_wifi_iface)"
  if [ -z "$iface" ]; then
    echo "WARN: no Wi‑Fi interface found; leaving cyclonedds.xml unchanged."
  else
    echo "Detected Wi‑Fi NIC: $iface"
    patch_cyclone_xml "$iface"
  fi
  echo "Peers: $JETSON_LAN_IP  $JETSON_WIFI_IP"
  echo

  ensure_docker_image
  echo
  install_sysctl_buffers
  echo
  install_bashrc
  QUIET=0 apply_env
  echo
  chmod +x "$GS_DIR/scripts/ensure_gs_container.sh" "$GS_DIR/scripts/run_joy_docker.sh" "$GS_DIR/scripts/stop_joy_docker.sh" 2>/dev/null || true
  bash "$GS_DIR/scripts/ensure_gs_container.sh"
  echo
  run_self_test
  print_done
}

bootstrap_native() {
  set -euo pipefail
  echo "=== AUV ground station setup (native ROS) ==="
  echo "GS_DIR=$GS_DIR"
  echo
  rm -f "$USE_DOCKER_MARKER"

  local iface
  iface="$(detect_wifi_iface)"
  if [ -z "$iface" ]; then
    echo "WARN: no Wi‑Fi interface found; leaving cyclonedds.xml unchanged."
  else
    echo "Detected Wi‑Fi NIC: $iface"
    patch_cyclone_xml "$iface"
  fi
  echo "Peers: $JETSON_LAN_IP  $JETSON_WIFI_IP"
  echo

  apt_install_ros
  echo
  install_sysctl_buffers
  echo
  build_joy
  echo
  install_bashrc
  QUIET=0 apply_env
  run_self_test
  print_done
}

# Sourced → env only. Executed → full setup (or --test only).
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  case "${1:-}" in
    --test|test)
      set -euo pipefail
      run_self_test
      ;;
    --native|native)
      bootstrap_native
      ;;
    --docker|docker|"")
      bootstrap_docker
      ;;
    *)
      echo "Usage: $0 [--docker|--native|--test]" >&2
      exit 1
      ;;
  esac
else
  apply_env
fi
