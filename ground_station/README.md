# Ground station

ROS 2 Humble **Docker image** for the operator PC. Used by MAV-GUI AppImage for the **PC acoustic modem (role a)**.

Xbox / joystick teleop is **not** here. The GUI uses the browser **Gamepad API → vehicle rosbridge** (`:9090`).

**Operator PC needs:** Docker, this image (`auv_gs` container), USB modem (`/dev/modem`).

```bash
./setup_env.sh --docker
```

Then in MAV-GUI → **Modem** → PC MODEM **On**.
