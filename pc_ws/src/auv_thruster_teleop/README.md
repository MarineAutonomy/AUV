# `auv_thruster_teleop`

Keyboard teleop for your 8-thruster AUV. Publishes PWM commands to ROS 2 topic **`/auv/thruster_cmd`** as `std_msgs/msg/Int32MultiArray` with **8 values**.

## Thruster order (matches Arduino serial parser)

The published array is in this order:

1. `psfr` (port-side front)
2. `sbfr` (starboard-side front)
3. `psaf` (port-side aft)
4. `sbaf` (starboard-side aft)
5. `psms1`
6. `sbms1`
7. `psms2`
8. `sbms2`

This matches `arduino/timi_pyserial/timi_pyserial.ino` `processMessage()`.

## Build (on your PC)

From the repo root:

```bash
cd pc_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --base-paths src
source install/setup.bash
```

## Run

```bash
cd pc_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run auv_thruster_teleop teleop_keyboard
```

Make sure your PC and Jetson have the same `ROS_DOMAIN_ID` and are on a network where DDS discovery works (WiFi or tether).

## Controls

### Deadman modes

By default, deadman uses **toggle** mode (easier than holding SPACE):

| Mode | How it works |
|---|---|
| `toggle` (default) | Press **SPACE** once to arm, press again to disarm |
| `hold` | Hold **SPACE** while commanding (classic deadman) |
| `off` | No deadman — thrust only while movement keys are held |

Change mode at launch:

```bash
ros2 run auv_thruster_teleop teleop_keyboard --ros-args -p deadman_mode:=off
```

Press **`0`** to instantly send neutral (also disarms in toggle mode).

| Key | Motion | Notes |
|---|---|---|
| `SPACE` | Arm / disarm | Toggle mode (default); hold in `hold` mode |
| `W` / `S` | Surge forward / back | XY-plane translation |
| `←` / `→` | Sway left / right | Requires X-config horizontal thrusters |
| `↑` / `↓` | Heave up / down | Drives all 4 vertical thrusters together |
| `A` / `D` | Yaw left / right | Uses horizontal thrusters differential |
| `0` | Neutral | Sends `[1500]*8` immediately |
| `CTRL-C` | Exit | Sends neutral on shutdown |

## Notes / tuning

- Default output range is **1500 ± 300** (clamped to **1100–1900**). You can tune via parameters:
  - `deadman_mode` (default `toggle`) — `hold`, `toggle`, or `off`
  - `scale_us` (default `300.0`)
  - `publish_hz` (default `20.0`)
  - `deadman_timeout_s` (default `0.25`, only used in `hold` mode)
  - `axis_timeout_s` (default `0.20`)

Example:

```bash
ros2 run auv_thruster_teleop teleop_keyboard --ros-args -p scale_us:=200.0 -p publish_hz:=30.0
```

