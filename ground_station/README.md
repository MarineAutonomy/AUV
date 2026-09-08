# Ground station (operator laptop)

Off-vehicle machine that runs **MAV-GUI**, joystick teleop, and joins the Jetson ROS graph over LAN/Wi‑Fi.

**Any new Ubuntu PC:** install/run MAV-GUI (below). You do not need to clone this repo or install ROS on the host — first launch sets up Docker + the joy image.

---

## Any Ubuntu PC (recommended)

1. Download the Linux release (`mav-gui` + AppImage, or the `.deb`).
2. Run **`./mav-gui`** (not the raw AppImage alone — that needs FUSE, which many PCs lack), or install the `.deb`.
3. First launch may show **Setting up ground station…** (password prompt for Docker). No clone / no `cd`.
4. Data under `~/.local/share/mav-gui/` (managed by the app).
5. Xbox tab → **Connect**.

```bash
chmod +x mav-gui mav-gui-*.AppImage
./mav-gui
# or:
sudo apt install ./mav-gui-*.deb
```

Skip auto-setup: `GS_SKIP_SETUP=1 ./mav-gui`

---

## Network assumptions

| Role | Address |
|------|---------|
| Jetson LAN | `192.168.194.10` (fixed) |
| Jetson Wi‑Fi | `192.168.0.50` — **reserve on the router** |

```bash
JETSON_WIFI_IP=192.168.0.42 ./setup_env.sh
```

---

## Manual / developer setup

Use this folder from the repo (or from `resources/ground_station/` in a packaged build) when you are not using the in-app setup.

```bash
cd ground_station
./setup_env.sh             # Docker (default) — no host ROS needed
./setup_env.sh --native    # apt install Humble + colcon on the host
./scripts/run_joy_docker.sh
```

`setup_env.sh --docker` will: detect Wi‑Fi NIC → `cyclonedds.xml`, install Docker if missing, pull or build `auv-ground-station:latest`, raise Cyclone UDP buffers, and optionally update `~/.bashrc`.

```bash
AUV_GS_IMAGE=myregistry/auv-ground-station:1.0 ./setup_env.sh
```

---

## Files

| Path | Purpose |
|------|---------|
| `setup_env.sh` | Setup (`--docker` default / `--native` / `--test`) |
| `Dockerfile` | ROS Humble + joy + `auv_joy_teleop` |
| `scripts/run_joy_docker.sh` | Start joy container (host net + `/dev/input`) |
| `cyclonedds.xml` | DDS: Wi‑Fi NIC + peers |
| `ros2_ws/src/auv_joy_teleop/` | Joy → `/auv/thruster_cmd` |
