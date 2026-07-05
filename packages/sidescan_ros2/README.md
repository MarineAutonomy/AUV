# sidescan_ros2

ROS2 Humble driver for the Cerulean Omniscan 450 side scan sonar.

Ported from the ROS1 `omniscan_sidescan` package, following patterns from `s500_ros2`.

## Build

```bash
cd /path/to/SIDESCAN_ROS2
colcon build --packages-select sidescan_ros2
source install/setup.bash
```

## Run

```bash
# 2-transducer (port + starboard)
ros2 launch sidescan_ros2 sidescan.launch.py

# 4-transducer (port + starboard + bow + stern)
ros2 launch sidescan_ros2 sidescan_all.launch.py
```

Override sonar IPs at launch:

```bash
ros2 launch sidescan_ros2 sidescan.launch.py \
  port_ip_address:=192.168.2.93 \
  starboard_ip_address:=192.168.2.95
```

## Topics

| Topic | Type |
|---|---|
| `/omniscan450/range` | `sidescan_ros2/msg/SideScanSonar` or `SideScanSonarAll` |
| `/omniscan450/range_raw` | `sidescan_ros2/msg/SideScanSonarRaw` or `SideScanSonarAllRaw` |

## Runtime parameters

Sonar tuning parameters (replacing ROS1 dynamic_reconfigure):

```bash
ros2 param set /omniscan450 gain_index 3
ros2 param set /omniscan450 length_mm 6000
```

See `config/omniscan450_params.yaml` for defaults.

## Dependencies

- ROS2 Humble
- `python3-serial` (used by vendored `brping`)
