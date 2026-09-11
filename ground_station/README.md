# Ground station (optional / legacy)

MAV-GUI Xbox teleop now uses the **browser Gamepad API → rosbridge** (Jetson `:9090`).
You do **not** need this Docker / `auv_joy_teleop` stack for normal joystick operation.

**Normal use:** run MAV-GUI → plug/pair Xbox → Xbox tab → **Connect** → press **X** to arm.

This folder remains only if you want the optional native ROS joy path over CycloneDDS again (`scripts/`, `ros2_ws/src/auv_joy_teleop/`).
