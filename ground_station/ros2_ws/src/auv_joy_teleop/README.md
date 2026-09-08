# auv_joy_teleop

Joystick → **same 8 PWM mix as MAV-GUI keyboard teleop** → `/auv/thruster_cmd`.

Mapping matches the AntiMicroX → keyboard layout used with the GUI.

## Controls (AntiMicroX / Xbox defaults)

| Input | Key equiv | Axis | Sign |
|-------|-----------|------|------|
| Left stick Y | W / S | surge | up = + |
| Left stick X | A / D | yaw | left = + |
| Right stick Y | ↑ / ↓ | heave | up = + |
| Right stick X | ← / → | roll | left = + |
| D-pad Y | R / F | pitch | up = nose down (−) |
| D-pad X | Q / E | sway | left = + |
| **X** | — | arm / disarm toggle |
| **B** | — | disarm + neutral |
| **A** | — | neutral while armed |

Publishes `/auv/joy_teleop/status` (`std_msgs/Bool`, armed) as a heartbeat so MAV-GUI can show **CONTROLLER** / **CONTROLLER ARMED** and lock GUI ARM while joy is running.

PWM order: `[PS-FR, SB-FR, PS-AF, SB-AF, PS-MS1, SB-MS1, PS-MS2, SB-MS2]`

## Run (ground station)

```bash
ros2 launch auv_joy_teleop joy_teleop.launch.py
```

Vehicle must be in **Auto**. Do not arm GUI teleop and joy at the same time.

If axes feel wrong, check indices:

```bash
ros2 topic echo /joy
```

Then edit `config/joy_teleop.yaml` and rebuild / reinstall.
