#!/usr/bin/env python3

import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


@dataclass
class AxisState:
    value: float = 0.0
    last_update_s: float = 0.0


class KeyboardReader:
    """
    Minimal raw-terminal key reader.
    Reads single characters from stdin in raw mode in a background thread.
    """

    def __init__(self) -> None:
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._last_key: Optional[str] = None
        self._last_key_time_s: float = 0.0

        self._stdin_fd = sys.stdin.fileno()
        self._old_term_settings = termios.tcgetattr(self._stdin_fd)

        tty.setraw(self._stdin_fd)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term_settings)
        except Exception:
            pass

    def pop_key(self) -> Optional[Tuple[str, float]]:
        with self._lock:
            if self._last_key is None:
                return None
            k, t = self._last_key, self._last_key_time_s
            self._last_key = None
            return (k, t)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            if not ch:
                continue
            key = self._decode_key(ch)
            if key is None:
                continue
            with self._lock:
                self._last_key = key
                self._last_key_time_s = time.time()

    def _decode_key(self, ch: str) -> Optional[str]:
        """
        Decode single characters + common escape sequences.

        Arrow keys arrive as:
          ESC [ A  (UP)
          ESC [ B  (DOWN)
          ESC [ C  (RIGHT)
          ESC [ D  (LEFT)
        """
        if ch != "\x1b":  # not ESC
            return ch

        try:
            nxt = sys.stdin.read(1)
            if nxt != "[":
                return None
            code = sys.stdin.read(1)
        except Exception:
            return None

        if code == "A":
            return "UP"
        if code == "B":
            return "DOWN"
        if code == "C":
            return "RIGHT"
        if code == "D":
            return "LEFT"
        return None


class ThrusterTeleop(Node):
    """
    Publishes 8 thruster PWM values to /auv/thruster_cmd.

    Arduino expects 8 comma-separated microseconds, in this order:
      [psfr, sbfr, psaf, sbaf, psms1, sbms1, psms2, sbms2]
    """

    def __init__(self) -> None:
        super().__init__("thruster_teleop_keyboard")

        # ROS parameters
        self.declare_parameter("topic", "/auv/thruster_cmd")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("deadman_mode", "toggle")  # hold | toggle | off
        self.declare_parameter("deadman_timeout_s", 0.25)
        self.declare_parameter("axis_timeout_s", 0.20)
        self.declare_parameter("scale_us", 300.0)  # max delta around 1500
        self.declare_parameter("pwm_min", 1100)
        self.declare_parameter("pwm_max", 1900)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value

        self._deadman_mode = (
            self.get_parameter("deadman_mode").get_parameter_value().string_value.lower()
        )
        if self._deadman_mode not in ("hold", "toggle", "off"):
            self.get_logger().warn(
                f"Unknown deadman_mode '{self._deadman_mode}', using 'toggle'"
            )
            self._deadman_mode = "toggle"

        self._deadman_timeout_s = (
            self.get_parameter("deadman_timeout_s").get_parameter_value().double_value
        )
        self._axis_timeout_s = self.get_parameter("axis_timeout_s").get_parameter_value().double_value
        self._scale_us = self.get_parameter("scale_us").get_parameter_value().double_value
        self._pwm_min = int(self.get_parameter("pwm_min").get_parameter_value().integer_value)
        self._pwm_max = int(self.get_parameter("pwm_max").get_parameter_value().integer_value)

        self._pub = self.create_publisher(Int32MultiArray, topic, 10)

        # Axis states (6DOF)
        self._axes: Dict[str, AxisState] = {
            "surge": AxisState(),
            "sway": AxisState(),
            "heave": AxisState(),
            "roll": AxisState(),
            "pitch": AxisState(),
            "yaw": AxisState(),
        }
        self._deadman_last_s: float = 0.0
        self._armed: bool = self._deadman_mode == "off"

        # Keymap (press-and-hold uses OS key-repeat; we timeout axes if no repeats)
        self._key_to_axis: Dict[str, Tuple[str, float]] = {
            "w": ("surge", +1.0),
            "s": ("surge", -1.0),
            "LEFT": ("sway", +1.0),
            "RIGHT": ("sway", -1.0),
            "UP": ("heave", +1.0),
            "DOWN": ("heave", -1.0),
            "a": ("yaw", +1.0),
            "d": ("yaw", -1.0),
            "t": ("pitch", +1.0),
            "g": ("pitch", -1.0),
            "z": ("roll", +1.0),
            "x": ("roll", -1.0),
        }

        deadman_help = {
            "hold": "Hold SPACE while commanding (release = neutral).",
            "toggle": "Press SPACE to arm/disarm (default).",
            "off": "No deadman — thrust only while movement keys are held.",
        }[self._deadman_mode]

        self.get_logger().info(
            f"Keyboard teleop started. {deadman_help}\n"
            "Keys: W/S surge, ←/→ sway, ↑/↓ heave, A/D yaw, T/G pitch, Z/X roll\n"
            "Other: SPACE=arm/disarm (toggle/hold), 0=all neutral, CTRL-C exits."
        )

        self._kbd = KeyboardReader()
        self._timer = self.create_timer(1.0 / max(1.0, publish_hz), self._tick)

    def destroy_node(self) -> bool:
        try:
            self._publish_neutral()
        except Exception:
            pass
        try:
            self._kbd.close()
        except Exception:
            pass
        return super().destroy_node()

    def _tick(self) -> None:
        now = time.time()

        # Consume latest key (1-char)
        key = self._kbd.pop_key()
        if key is not None:
            ch, t = key
            if ch == " ":
                if self._deadman_mode == "hold":
                    self._deadman_last_s = t
                elif self._deadman_mode == "toggle":
                    self._armed = not self._armed
                    state = "ARMED" if self._armed else "DISARMED"
                    self.get_logger().info(f"Teleop {state}")
                    if not self._armed:
                        self._publish_neutral()
            elif ch == "0":
                self._publish_neutral()
                for a in self._axes.values():
                    a.value = 0.0
                    a.last_update_s = now
                if self._deadman_mode == "toggle":
                    self._armed = False
                    self.get_logger().info("Teleop DISARMED")
                return
            elif ch in self._key_to_axis:
                axis, direction = self._key_to_axis[ch]
                st = self._axes[axis]
                st.value = float(direction)
                st.last_update_s = t

        # Timeout axis commands back to zero (so releasing key returns neutral)
        for st in self._axes.values():
            if now - st.last_update_s > self._axis_timeout_s:
                st.value = 0.0

        if self._deadman_mode == "hold":
            deadman_ok = (now - self._deadman_last_s) <= self._deadman_timeout_s
        elif self._deadman_mode == "toggle":
            deadman_ok = self._armed
        else:
            deadman_ok = True

        if not deadman_ok:
            self._publish_neutral()
            return

        pwm = self._mix_to_pwm()
        msg = Int32MultiArray()
        msg.data = pwm
        self._pub.publish(msg)

    def _mix_to_pwm(self) -> list:
        """
        Default mixer:
          - First 4 thrusters: surge + sway + yaw (generic 4-thruster horizontal mix)
          - Last 4 thrusters: heave only (same command)

        Note: exact sway effectiveness depends on your horizontal thruster geometry.
        """
        surge = self._axes["surge"].value
        sway = self._axes["sway"].value
        heave = self._axes["heave"].value
        roll = self._axes["roll"].value  # default unused
        pitch = self._axes["pitch"].value  # default unused
        yaw = self._axes["yaw"].value

        def clamp01(x: float) -> float:
            return max(-1.0, min(1.0, x))

        # Horizontal (psfr, sbfr, psaf, sbaf)
        # Generic 4-thruster planar mix (surge/sway/yaw).
        t0 = clamp01(surge + sway + yaw)
        t1 = clamp01(surge - sway - yaw)
        t2 = clamp01(surge - sway + yaw)
        t3 = clamp01(surge + sway - yaw)

        # Vertical group (psms1, sbms1, psms2, sbms2)
        # Current Arduino code drives these four together (CH3).
        v = clamp01(heave)

        # Convert to PWM microseconds
        def to_pwm(u: float) -> int:
            return int(round(1500 + self._scale_us * u))

        pwm = [to_pwm(t0), to_pwm(t1), to_pwm(t2), to_pwm(t3)]
        pwm += [to_pwm(v), to_pwm(v), to_pwm(v), to_pwm(v)]

        # Clamp to safe range
        pwm = [max(self._pwm_min, min(self._pwm_max, p)) for p in pwm]
        return pwm

    def _publish_neutral(self) -> None:
        msg = Int32MultiArray()
        msg.data = [1500] * 8
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

