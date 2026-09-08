#!/usr/bin/env python3
"""Subscribe /joy and publish /auv/thruster_cmd with the same mix as MAV-GUI teleop."""

import json
import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32MultiArray

from auv_joy_teleop.thruster_mix import clamp01, mix_to_pwm, neutral_pwm

STATUS_TOPIC = "/auv/joy_teleop/status"
BAR_PWM_FILE = "/tmp/mav_gui_bar_pwm.json"
STATUS_FILE = "/tmp/mav_gui_joy_status.json"


def _axis(msg: Joy, index: int, default: float = 0.0) -> float:
    if index < 0 or index >= len(msg.axes):
        return default
    v = float(msg.axes[index])
    return default if math.isnan(v) else v


def _button(msg: Joy, index: int) -> int:
    if index < 0 or index >= len(msg.buttons):
        return 0
    return int(msg.buttons[index])


def _apply_deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    # Rescale so motion starts at deadzone edge.
    sign = 1.0 if v > 0.0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


class JoyThrusterTeleop(Node):
    def __init__(self) -> None:
        super().__init__("joy_thruster_teleop")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("thruster_topic", "/auv/thruster_cmd")
        self.declare_parameter("status_topic", STATUS_TOPIC)
        self.declare_parameter("deadzone", 0.04)
        self.declare_parameter("scale_us", 300.0)
        self.declare_parameter("pwm_min", 1100)
        self.declare_parameter("pwm_max", 1900)
        self.declare_parameter("require_arm", True)
        # Heartbeat / hold-last-command rate. Stick motion publishes immediately in _on_joy.
        self.declare_parameter("publish_hz", 50.0)

        # AntiMicroX ↔ GUI keys (override via joy_teleop.yaml)
        # Stick1: Y surge W/S, X yaw A/D | Stick2: Y heave ↑/↓, X roll ←/→
        # D-pad: Y pitch R/F, X sway Q/E
        self.declare_parameter("axis_surge", 1)  # left stick Y
        self.declare_parameter("axis_yaw", 0)  # left stick X
        self.declare_parameter("axis_heave", 4)  # right stick Y
        self.declare_parameter("axis_roll", 3)  # right stick X
        self.declare_parameter("axis_pitch", 7)  # d-pad Y
        self.declare_parameter("axis_sway", 6)  # d-pad X
        self.declare_parameter("axis_heave_pos", -1)  # unused
        self.declare_parameter("axis_heave_neg", -1)

        # Stick-up / d-pad-left often report −1; invert to match GUI key signs
        self.declare_parameter("invert_surge", True)  # up → W surge +
        self.declare_parameter("invert_yaw", True)  # left → A yaw +
        self.declare_parameter("invert_heave", True)  # up → ↑ heave +
        self.declare_parameter("invert_roll", True)  # left → ← roll +
        self.declare_parameter("invert_pitch", False)  # up (−1) → R pitch − (nose down)
        self.declare_parameter("invert_sway", True)  # left → Q sway +

        # Buttons (Xbox-style defaults)
        self.declare_parameter("button_arm_toggle", 2)  # X
        self.declare_parameter("button_arm_hold", -1)  # optional hold-to-arm (−1 = unused)
        self.declare_parameter("button_disarm", 1)  # B
        self.declare_parameter("button_neutral", 0)  # A → snap neutral while armed

        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value
        thruster_topic = self.get_parameter("thruster_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value

        self._armed = False
        self._prev_arm_btn = 0
        self._last_joy: Joy | None = None
        self._last_status_write_ns = 0

        self._pub = self.create_publisher(Int32MultiArray, thruster_topic, 10)
        self._status_pub = self.create_publisher(Bool, status_topic, 10)
        # Sensor data: best-effort + small depth keeps the latest stick sample.
        joy_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Joy, joy_topic, self._on_joy, joy_qos)

        hz = float(self.get_parameter("publish_hz").value)
        period = 1.0 / max(1.0, hz)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Joy teleop: {joy_topic} → {thruster_topic} "
            f"(status {status_topic}; arm with X; B disarm; same mix as MAV-GUI)"
        )
        self._publish_status()

    def _publish_status(self, force_file: bool = False) -> None:
        msg = Bool()
        msg.data = bool(self._armed)
        self._status_pub.publish(msg)
        # Local GUI reads this file (PC only — not Jetson/rosbridge).
        # Throttle disk writes; arm edges always flush.
        self._write_status_file(force=force_file)

    def _write_status_file(self, force: bool = False) -> None:
        now = self.get_clock().now().nanoseconds
        if not force and (now - self._last_status_write_ns) < 50_000_000:  # 50 ms
            return
        self._last_status_write_ns = now
        try:
            payload = json.dumps({"armed": bool(self._armed), "running": True})
            tmp = STATUS_FILE + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(payload)
            os.replace(tmp, STATUS_FILE)
        except OSError:
            pass

    def _set_armed(self, armed: bool, reason: str = "") -> None:
        if self._armed == armed:
            return
        self._armed = armed
        if armed:
            self.get_logger().info(f"ARMED{(' — ' + reason) if reason else ''}")
        else:
            self.get_logger().info(f"DISARMED → neutral{(' — ' + reason) if reason else ''}")
            self._publish(neutral_pwm())
        self._publish_status(force_file=True)

    def _signed_axis(self, msg: Joy, name: str) -> float:
        idx = int(self.get_parameter(f"axis_{name}").value)
        inv = bool(self.get_parameter(f"invert_{name}").value)
        dz = float(self.get_parameter("deadzone").value)
        v = _apply_deadzone(_axis(msg, idx), dz)
        return -v if inv else v

    def _heave_from_joy(self, msg: Joy) -> float:
        # Prefer dedicated heave axis; if unused/zero, use LT/RT analog if configured.
        heave = self._signed_axis(msg, "heave")
        if abs(heave) > 1e-3:
            return heave

        dz = float(self.get_parameter("deadzone").value)
        rt_idx = int(self.get_parameter("axis_heave_pos").value)
        lt_idx = int(self.get_parameter("axis_heave_neg").value)
        rt = _axis(msg, rt_idx, default=0.0)
        lt = _axis(msg, lt_idx, default=0.0)

        def trigger_amount(x: float) -> float:
            if x < 0.0:
                return _apply_deadzone((x + 1.0) * 0.5, dz * 0.5)
            return _apply_deadzone(x, dz)

        return clamp01(trigger_amount(rt) - trigger_amount(lt))

    def _on_joy(self, msg: Joy) -> None:
        arm_idx = int(self.get_parameter("button_arm_toggle").value)
        disarm_idx = int(self.get_parameter("button_disarm").value)
        neutral_idx = int(self.get_parameter("button_neutral").value)
        hold_idx = int(self.get_parameter("button_arm_hold").value)

        arm_btn = _button(msg, arm_idx)
        if arm_btn == 1 and self._prev_arm_btn == 0:
            self._set_armed(not self._armed, "X")
        self._prev_arm_btn = arm_btn

        if _button(msg, disarm_idx) == 1:
            self._set_armed(False, "B")

        if hold_idx >= 0:
            self._set_armed(_button(msg, hold_idx) == 1, "hold")

        if self._armed and _button(msg, neutral_idx) == 1:
            self._publish(neutral_pwm())
            self._write_bar_file(neutral_pwm())
            self._last_joy = msg
            return

        self._last_joy = msg
        values = self._mix_from_joy(msg)
        # GUI bar file + vehicle cmd on the stick event (not waiting for the timer).
        self._write_bar_file(values)
        require_arm = bool(self.get_parameter("require_arm").value)
        if (not require_arm) or self._armed:
            self._publish(values)

    def _mix_from_joy(self, msg: Joy) -> list[int]:
        scale = float(self.get_parameter("scale_us").value)
        pwm_min = int(self.get_parameter("pwm_min").value)
        pwm_max = int(self.get_parameter("pwm_max").value)
        return mix_to_pwm(
            self._signed_axis(msg, "surge"),
            self._signed_axis(msg, "sway"),
            self._heave_from_joy(msg),
            self._signed_axis(msg, "yaw"),
            self._signed_axis(msg, "pitch"),
            self._signed_axis(msg, "roll"),
            scale_us=scale,
            pwm_min=pwm_min,
            pwm_max=pwm_max,
        )

    def _on_timer(self) -> None:
        # Heartbeat so the GUI knows joy teleop is running.
        self._publish_status()

        if self._last_joy is None:
            self._write_bar_file(neutral_pwm())
            return

        values = self._mix_from_joy(self._last_joy)
        self._write_bar_file(values)

        require_arm = bool(self.get_parameter("require_arm").value)
        if require_arm and not self._armed:
            return
        # Hold last command while sticks are idle (joy_node may stop publishing).
        self._publish(values)

    def _write_bar_file(self, values: list[int]) -> None:
        try:
            payload = json.dumps([int(v) for v in values])
            tmp = BAR_PWM_FILE + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(payload)
            os.replace(tmp, BAR_PWM_FILE)
        except OSError:
            pass

    def _publish(self, values: list[int]) -> None:
        out = Int32MultiArray()
        out.data = [int(v) for v in values]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyThrusterTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._set_armed(False, "shutdown")
            node._publish(neutral_pwm())
            try:
                os.unlink(STATUS_FILE)
            except OSError:
                pass
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
