# AUV

Monorepo for **autonomous underwater vehicle (AUV)** software: ROS 2 guidance, navigation, and control (GNC), sensor bring-up packages, optional desktop operator GUI, Arduino / micro-ROS experiments, and Jetson-focused operational notes. The stack targets **ROS 2 Humble** on Ubuntu (and Jetson L4T via the provided dev container).

---

## What lives in this repository

| Area | Purpose |
|------|---------|
| [`code_ws/`](code_ws/) | **Primary ROS 2 colcon workspace** — Python `ament_python` packages for 2D/3D control, missions, navigation EKF, modem bridge, and Arduino serial helpers. |
| [`packages/`](packages/) | **Sensor and vendor ROS 2 packages** (DVL, Ping sonar, SBG driver, message definitions). Used as `src` for a separate sensor workspace inside the dev container (`~/ros2_ws`). |
| [`arduino/`](arduino/) | Sketches and tests (e.g. micro-ROS, SAUVC-related, serial experiments). |
| [`.devcontainer/`](.devcontainer/) | Docker-based dev image: ROS 2 Humble, ZED SDK base, colcon, Python venv, Arduino CLI, micro-ROS Arduino library, pre-build of `packages/` into `~/ros2_ws`. |
| [`system_setups/`](system_setups/) | Operational runbooks (e.g. Jetson Wi-Fi, SSH, Docker, DVL SSH tunnel, DDS tuning). |
| [`kf_tests/`](kf_tests/) | Offline **EKF replay / evaluation** against ROS 2 bags using the same filter core as the `navigation` package. |
| [`requirements.txt`](requirements.txt) | Python dependencies used in the container / lab environment (NumPy, SciPy, plotting, serial, modem-related libs, etc.). |

---

## ROS 2 workspaces

### 1. Application workspace — `code_ws`


**Packages (ament_python)**

| ROS package / folder | Role | Main executables (examples) |
|----------------------|------|-----------------------------|
| **`auv_2d_control`** | Planar control: surge/sway and yaw (`u`, `v`, yaw). | `controller_2d`, `thruster_allocator_2d` |
| **`auv_3d_control`** | Full 6-DOF-style control (`u`, `v`, `w`, roll, pitch, yaw). | `controller_3d`, `thruster_allocator_3d` |
| **`auv_2d_missions`** | 2D mission guidance (e.g. point tracking). | `point_tracking_mission_2d_los`, `point_tracking_mission_2d_ilos` |
| **`auv_3d_missions`** | 3D mission guidance. | `point_tracking_mission_3d_los`, `point_tracking_mission_3d_ilos` |
| **`auv_navigation`** (ROS name: **`navigation`**) | **EKF navigation** — IMU, DVL, dead-reckoning, depth, optional range/encoders; numerically stable update, optional Mahalanobis gating. | `navigation_node` |
| **`modem_m64`** | Acoustic modem (Water Linked M64) ROS node. | `modem_node` |
| **`arduino_ps`** | Bridge / utilities related to Arduino + ROS (package metadata still generic). | `arduino_ps` |

**Dependency note:** `code_ws/src/dvl_msgs` is typically a **symlink** into [`packages/dvl_msgs`](packages/dvl_msgs) so the navigation stack can build against DVL message definitions without duplicating the package.

**Example — 2D control stack**

```bash
source install/setup.bash
ros2 launch auv_2d_control control_2d.launch.py
```

Parameters default to `auv_2d_control/config/control_2d_default.yaml`; override with `params_file:=/path/to/your.yaml`.

**Example — navigation node**

Configuration follows `auv_navigation/config/vessel_data.example.yml`. Run with an explicit vessel file or `AUV_VESSEL_DATA`:

```bash
ros2 run navigation navigation_node --ros-args -p vessel_data_file:=/absolute/path/to/vessel_data.yml
```

The filter expects the **same topic contract** documented in that example (SBG IMU topics, `dvl_msgs`, depth, etc., depending on what you enable in YAML).

### 2. Sensor / driver workspace — `packages/`

These packages are maintained under [`packages/`](packages/) and, in the **dev container Dockerfile**, are copied to `/home/mavlab/ros2_ws/src` and built once as **`~/ros2_ws/install`**. That keeps heavy or third-party drivers (e.g. **SBG**, **Nortek DVL A50**, **Blue Robotics Ping / Ping360**) in a dedicated workspace while `code_ws` holds vehicle-specific GNC.

On a bare machine you can create your own workspace that includes `packages/` (and symlink `dvl_msgs` into `code_ws` as this repo does).

---

## Missions and controllers (high level)

- **Missions** (`auv_*_missions`) implement guidance laws (e.g. **LOS** / **ILOS** point tracking) and publish setpoints or references consumed by the controllers.
- **Controllers** (`auv_*_control`) close the loop and work with **thruster allocators** to map body-frame demands to actuator commands.
- **Navigation** (`navigation` from `auv_navigation`) fuses sensors into state/odometry for closed-loop autonomy and logging.

YAML under each package’s `config/` directory tunes gains, targets, and mission behaviour.

---

## Running missions (typical onboard order)

Run these as **separate processes** (separate terminals, systemd units, or a composed launch you add yourself). Order matters: the Arduino bridge must publish depth (and related topics) before the EKF can use them; the controller must be listening for references before the mission starts publishing them.


Bring up any **sensors declared in your vessel YAML** (e.g. SBG IMU, DVL) so `navigation_node` actually receives data.

**1. Arduino bridge — `arduino_ps`**

Publishes `/auv/depth`, pressure, temperature, and forwards `/auv/thruster_cmd` to the MCU. Default serial is `/dev/arduino_mega` @ `115200`; override if your device differs.

```bash
ros2 run arduino_ps arduino_ps --ros-args -p port:=/dev/arduino_mega -p baud:=115200
```

**2. Navigation — EKF**

Use a vessel file copied from the example and edited for your vehicle (see `auv_navigation/config/vessel_data.example.yml`).

```bash
ros2 run navigation navigation_node --ros-args -p vessel_data_file:=/absolute/path/to/vessel_data.yml
```

Alternatively set **`AUV_VESSEL_DATA`** to that path and omit the parameter if your deployment relies on that env var (check `navigation_node` for supported overrides).

**3. Control — 2D or 3D launch**

2D (planar: `u`, `v`, yaw):

```bash
ros2 launch auv_2d_control control_2d.launch.py
# optional: params_file:=/path/to/your_control_2d.yaml
```

3D:

```bash
ros2 launch auv_3d_control control_3d.launch.py
# optional: params_file:=/path/to/your_control_3d.yaml
```

Pick **either** 2D **or** 3D to match the mission package you run next.

**4. Mission node**

2D LOS example (loads share-installed YAML):

```bash
ros2 run auv_2d_missions point_tracking_mission_2d_los --ros-args \
  --params-file "$(ros2 pkg prefix auv_2d_missions)/share/auv_2d_missions/config/point_tracking/point_tracking_los.yaml"
```

Other entry points from the same package: `point_tracking_mission_2d_ilos`; for 3D missions use `auv_3d_missions` and `point_tracking_mission_3d_los` / `point_tracking_mission_3d_ilos` with the matching files under `auv_3d_missions/config/point_tracking/`.

Missions default to odometry topic **`navigation/odometry`** and publish references on **`/guidance/reference`** (overridable via node parameters in the mission source).

---

## Dev container

- **Definition:** [`.devcontainer/devcontainer.json`](.devcontainer/devcontainer.json) + [`.devcontainer/Dockerfile`](.devcontainer/Dockerfile).
- **Base image:** Stereolabs Zed SDK image for **JetPack 6.2 / L4T r36.4**, extended with **ROS 2 Humble**, build tools, RViz, rosbridge, Foxglove bridge, Cyclone DDS RMW, image/point-cloud transports, etc.
- **Privileges:** Container is intended to run with **`--privileged`** and **`--network=host`** for hardware and DDS behaviour typical of robotics setups.
- **Workspaces inside the container:** The Dockerfile assumes the repo is mounted at something like **`/workspaces/mavlab`** with **`code_ws`** under it; `bashrc` aliases source `/opt/ros/humble`, `~/ros2_ws/install`, and optionally `/workspaces/mavlab/code_ws/install`.

Supporting files expected next to the Dockerfile (see comments in the Dockerfile): e.g. `requirements.txt`, `micro_ros_arduino_arm.zip`, and the `packages/` tree used during `docker build`.

---


## Offline EKF testing — `kf_tests`

[`kf_tests/test_ekf.py`](kf_tests/test_ekf.py) replays ROS 2 bags, runs the **same EKF implementation** as the live node (imported from the `navigation` Python package under `auv_navigation`), and writes plots plus a text summary under e.g. `kf_tests/run3/`.

```bash
cd kf_tests
python3 test_ekf.py run3
```

Requirements include **rosbags**, **NumPy**, **SciPy**, **Matplotlib**, **PyYAML** (install in your venv as needed).  
**Note:** The script inserts `code_ws/src/navigation` on `sys.path`, but the actual directory in this repo is **`code_ws/src/auv_navigation`** (the importable module is still `navigation`). If imports fail, adjust `NAV_SRC` in `test_ekf.py` to point at `auv_navigation`.

---

## Jetson and field setup

See [`system_setups/jetson_auv_setup.md`](system_setups/jetson_auv_setup.md) for Wi-Fi, SSH, Docker, USB Wi-Fi drivers, static Ethernet to a switch, **DVL** access via SSH port forward, and **Cyclone DDS** tuning for large messages.

---


## Maintainer / license

Core ROS packages in `code_ws` declare maintainer **aatmaj** and license **Apache-2.0** where `package.xml` is filled in. Third-party content under `packages/` retains upstream licenses (see each package’s `README` or `package.xml`).

---

