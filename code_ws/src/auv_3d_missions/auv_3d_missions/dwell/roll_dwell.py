"""DWELL mission: initial hold, then N cycles of (hold 10s + roll -45° → 0° → 45°)."""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def yaw_from_quat(q: Quaternion) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Phase(Enum):
    WAIT_ODOM = auto()
    INITIAL_HOLD = auto()
    CYCLE_HOLD = auto()
    ROLL_NEG45 = auto()
    ROLL_0 = auto()
    ROLL_POS45 = auto()
    DONE = auto()


# One roll cycle sequence after the per-cycle station-keep.
_ROLL_SEQ_DEG = (-45.0, 0.0, 45.0)


class DwellMission3d(Node):
    """
    Hold NED (x, y, z) with pitch/yaw setpoints.

    1) Station-keep for initial_hold_s (roll = 0°)
    2) Repeat `cycles` times:
         - station-keep cycle_hold_s (default 10 s, roll = 0°)
         - roll to -45°, 0°, +45° (each for roll_hold_s)
    3) Finish holding last pose (roll = +45°)

    Publishes [u_d, v_d, w_d, phi_d, theta_d, psi_d] on /guidance/reference_3d.
    """

    def __init__(self) -> None:
        super().__init__("dwell_mission_3d")

        self.declare_parameter("odom_topic", "navigation/odometry")
        self.declare_parameter("reference_topic", "/guidance/reference_3d")
        self.declare_parameter("status_topic", "/mission/status")

        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 1.0)
        self.declare_parameter("target_pitch_deg", 0.0)
        self.declare_parameter("target_yaw_deg", 0.0)

        self.declare_parameter("initial_hold_s", 10.0)
        self.declare_parameter("cycles", 1)
        self.declare_parameter("cycle_hold_s", 10.0)
        self.declare_parameter("roll_hold_s", 4.0)

        self.declare_parameter("kp_xy", 0.35)
        self.declare_parameter("u_max", 0.30)
        self.declare_parameter("v_max", 0.25)
        self.declare_parameter("w_gain", 0.70)
        self.declare_parameter("w_max", 0.25)
        self.declare_parameter("pos_tolerance", 0.15)

        odom_topic = str(self.get_parameter("odom_topic").value)
        reference_topic = str(self.get_parameter("reference_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self._x: Optional[float] = None
        self._y: Optional[float] = None
        self._z: Optional[float] = None
        self._psi: Optional[float] = None

        self._phase = Phase.WAIT_ODOM
        self._phase_t0: Optional[float] = None
        self._cycle_i = 0
        self._roll_cmd_deg = 0.0

        self._ref_pub = self.create_publisher(Float64MultiArray, reference_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._timer = self.create_timer(0.1, self._on_timer)

        tx = float(self.get_parameter("target_x").value)
        ty = float(self.get_parameter("target_y").value)
        tz = float(self.get_parameter("target_z").value)
        n = int(self.get_parameter("cycles").value)
        t0 = float(self.get_parameter("initial_hold_s").value)
        self.get_logger().info(
            f"dwell_mission_3d ready — target=({tx:.3f},{ty:.3f},{tz:.3f}) "
            f"initial_hold={t0:.1f}s cycles={n} "
            f"(cycle: hold {float(self.get_parameter('cycle_hold_s').value):.0f}s then roll -45/0/+45)"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _enter(self, phase: Phase) -> None:
        self._phase = phase
        self._phase_t0 = self._now()

    def _elapsed(self) -> float:
        if self._phase_t0 is None:
            return 0.0
        return max(0.0, self._now() - self._phase_t0)

    def _on_odom(self, msg: Odometry) -> None:
        self._x = float(msg.pose.pose.position.x)
        self._y = float(msg.pose.pose.position.y)
        self._z = float(msg.pose.pose.position.z)
        self._psi = yaw_from_quat(msg.pose.pose.orientation)

    def _publish_status(self, text: str) -> None:
        m = String()
        m.data = text
        self._status_pub.publish(m)

    def _publish_reference(
        self, u_d: float, v_d: float, w_d: float, phi_d: float, theta_d: float, psi_d: float
    ) -> None:
        m = Float64MultiArray()
        m.data = [u_d, v_d, w_d, phi_d, theta_d, psi_d]
        self._ref_pub.publish(m)

    def _hold_commands(self) -> tuple[float, float, float, float, float, float]:
        tx = float(self.get_parameter("target_x").value)
        ty = float(self.get_parameter("target_y").value)
        tz = float(self.get_parameter("target_z").value)
        theta_d = math.radians(float(self.get_parameter("target_pitch_deg").value))
        psi_d = wrap_pi(math.radians(float(self.get_parameter("target_yaw_deg").value)))
        phi_d = math.radians(self._roll_cmd_deg)
        kp_xy = float(self.get_parameter("kp_xy").value)
        u_max = float(self.get_parameter("u_max").value)
        v_max = float(self.get_parameter("v_max").value)
        w_gain = float(self.get_parameter("w_gain").value)
        w_max = float(self.get_parameter("w_max").value)

        assert self._x is not None and self._y is not None and self._z is not None
        assert self._psi is not None
        ex_n = tx - self._x
        ey_e = ty - self._y
        ez = self._z - tz
        psi = float(self._psi)
        c, s = math.cos(psi), math.sin(psi)
        ex_b = c * ex_n + s * ey_e
        ey_b = -s * ex_n + c * ey_e
        u_d = clamp(kp_xy * ex_b, -u_max, u_max)
        v_d = clamp(kp_xy * ey_b, -v_max, v_max)
        w_d = clamp(-w_gain * ez, -w_max, w_max)
        return u_d, v_d, w_d, phi_d, theta_d, psi_d

    def _advance_fsm(self) -> None:
        cycles = max(0, int(self.get_parameter("cycles").value))
        initial_hold = max(0.0, float(self.get_parameter("initial_hold_s").value))
        cycle_hold = max(0.0, float(self.get_parameter("cycle_hold_s").value))
        roll_hold = max(0.5, float(self.get_parameter("roll_hold_s").value))
        t = self._elapsed()

        if self._phase == Phase.WAIT_ODOM:
            self._enter(Phase.INITIAL_HOLD)
            self._roll_cmd_deg = 0.0
            self._cycle_i = 0
            return

        if self._phase == Phase.INITIAL_HOLD:
            self._roll_cmd_deg = 0.0
            if t >= initial_hold:
                if cycles <= 0:
                    self._enter(Phase.DONE)
                else:
                    self._enter(Phase.CYCLE_HOLD)
            return

        if self._phase == Phase.CYCLE_HOLD:
            self._roll_cmd_deg = 0.0
            if t >= cycle_hold:
                self._enter(Phase.ROLL_NEG45)
                self._roll_cmd_deg = _ROLL_SEQ_DEG[0]
            return

        if self._phase == Phase.ROLL_NEG45:
            self._roll_cmd_deg = _ROLL_SEQ_DEG[0]
            if t >= roll_hold:
                self._enter(Phase.ROLL_0)
                self._roll_cmd_deg = _ROLL_SEQ_DEG[1]
            return

        if self._phase == Phase.ROLL_0:
            self._roll_cmd_deg = _ROLL_SEQ_DEG[1]
            if t >= roll_hold:
                self._enter(Phase.ROLL_POS45)
                self._roll_cmd_deg = _ROLL_SEQ_DEG[2]
            return

        if self._phase == Phase.ROLL_POS45:
            self._roll_cmd_deg = _ROLL_SEQ_DEG[2]
            if t >= roll_hold:
                self._cycle_i += 1
                if self._cycle_i >= cycles:
                    self._enter(Phase.DONE)
                else:
                    self._enter(Phase.CYCLE_HOLD)
                    self._roll_cmd_deg = 0.0
            return

        # DONE: keep holding last commanded roll
        return

    def _on_timer(self) -> None:
        if self._x is None or self._y is None or self._z is None or self._psi is None:
            self._publish_status("dwell_3d: waiting for odometry")
            return

        if self._phase == Phase.WAIT_ODOM:
            self._advance_fsm()

        self._advance_fsm()
        u_d, v_d, w_d, phi_d, theta_d, psi_d = self._hold_commands()
        self._publish_reference(u_d, v_d, w_d, phi_d, theta_d, psi_d)

        cycles = max(0, int(self.get_parameter("cycles").value))
        phase = self._phase.name
        self._publish_status(
            f"dwell_3d: {phase} cycle={min(self._cycle_i + 1, max(cycles, 1))}/{cycles} "
            f"roll={self._roll_cmd_deg:.0f}° t={self._elapsed():.1f}s "
            f"pos=({self._x:.2f},{self._y:.2f},{self._z:.2f})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DwellMission3d()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
