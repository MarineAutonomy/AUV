# FS450 ROS2 DRIVER

ROS2 Humble driver for the Cerulean Omniscan 450 **FS** (forward-scan) sonar.

Uses the same Cerulean Ping Protocol and `Omniscan450` API (`os_mono_profile` over TCP port 51200).

## Build

```bash
colcon build --packages-select frontscan_ros2
source install/setup.bash
```

## Run driver only

```bash
ros2 launch frontscan_ros2 frontscan.launch.py
# optional override: ip_address:=192.168.194.90
```

## Run driver + rosbridge + web viewer

```bash
ros2 launch frontscan_ros2 frontscan_viewer.launch.py
```

Open: http://localhost:9002/

Requires `ros-humble-rosbridge-server` for the viewer launch file.

## Topics

| Topic | Type | Description |
|---|---|---|
| `/frontscan450/range` | `frontscan_ros2/msg/FrontScanSonar` | Scaled dB forward range profile |
| `/frontscan450/range_raw` | `frontscan_ros2/msg/FrontScanSonarRaw` | Raw uint16 power values |

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `ip_address` | `192.168.194.90` | Sonar IP |
| `sonar_port` | `51200` | TCP port |
| `web_port` | `9002` | Web viewer HTTP port (viewer launch only) |
| `rosbridge_port` | `9090` | Rosbridge WebSocket port (viewer launch only) |

## Runtime parameters

```bash
ros2 param set /frontscan450 gain_index 3
ros2 param set /frontscan450 length_mm 6000
```

See `config/omniscan450_fs_params.yaml` for defaults.

## Dependencies

- ROS2 Humble
- `python3-serial` (vendored `brping`)
- `ros-humble-rosbridge-server` (optional, for web viewer)

## Notes

- Each message is a **1D forward range profile** (not a full 2D image per ping).
- The web viewer shows a forward **history plot** (range vs time) plus the latest ping profile.
- SonarView-style 2D compositing would require stacking pings with vehicle pose (future work).

## License

MIT
