"""Station-keeping mission: hold a fixed NED (x, y, z) point."""

from __future__ import annotations

import math
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


class StationKeepingMission3d(Node):
    """
    Hold a 3D NED setpoint (x, y, z).

    Body-frame P correction on horizontal error (u_d, v_d), depth loop like
    depth-control for w_d / theta_d, heading held from first odom sample.
    Publishes [u_d, v_d, w_d, phi_d, theta_d, psi_d] on /guidance/reference_3d.
    """

    def __init__(self) -> None:
        super().__init__("station_keeping_mission_3d")

        self.declare_parameter("odom_topic", "navigation/odometry")
        self.declare_parameter("reference_topic", "/guidance/reference_3d")
        self.declare_parameter("status_topic", "/mission/status")

        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 1.0)

        self.declare_parameter("kp_xy", 0.35)
        self.declare_parameter("u_max", 0.30)
        self.declare_parameter("v_max", 0.25)
        self.declare_parameter("w_gain", 0.70)
        self.declare_parameter("w_max", 0.25)
        self.declare_parameter("lookahead_z", 0.70)
        self.declare_parameter("pos_tolerance", 0.15)

        odom_topic = str(self.get_parameter("odom_topic").value)
        reference_topic = str(self.get_parameter("reference_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self._x: Optional[float] = None
        self._y: Optional[float] = None
        self._z: Optional[float] = None
        self._psi_hold: Optional[float] = None

        self._ref_pub = self.create_publisher(Float64MultiArray, reference_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._timer = self.create_timer(0.1, self._on_timer)

        tx = float(self.get_parameter("target_x").value)
        ty = float(self.get_parameter("target_y").value)
        tz = float(self.get_parameter("target_z").value)
        self.get_logger().info(
            f"station_keeping_mission_3d ready — target=({tx:.3f}, {ty:.3f}, {tz:.3f}) NED"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._x = float(msg.pose.pose.position.x)
        self._y = float(msg.pose.pose.position.y)
        self._z = float(msg.pose.pose.position.z)
        if self._psi_hold is None:
            self._psi_hold = yaw_from_quat(msg.pose.pose.orientation)

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

    def _on_timer(self) -> None:
        if self._x is None or self._y is None or self._z is None or self._psi_hold is None:
            return

        tx = float(self.get_parameter("target_x").value)
        ty = float(self.get_parameter("target_y").value)
        tz = float(self.get_parameter("target_z").value)
        kp_xy = float(self.get_parameter("kp_xy").value)
        u_max = float(self.get_parameter("u_max").value)
        v_max = float(self.get_parameter("v_max").value)
        w_gain = float(self.get_parameter("w_gain").value)
        w_max = float(self.get_parameter("w_max").value)
        look_z = float(self.get_parameter("lookahead_z").value)
        tol = float(self.get_parameter("pos_tolerance").value)

        # NED position error toward target (north, east); depth error like depth-control.
        ex_n = tx - self._x
        ey_e = ty - self._y
        ez = self._z - tz

        psi = float(self._psi_hold)
        c, s = math.cos(psi), math.sin(psi)
        # Body: x forward, y starboard
        ex_b = c * ex_n + s * ey_e
        ey_b = -s * ex_n + c * ey_e

        u_d = clamp(kp_xy * ex_b, -u_max, u_max)
        v_d = clamp(kp_xy * ey_b, -v_max, v_max)
        w_d = clamp(-w_gain * ez, -w_max, w_max)
        theta_d = math.atan2(-ez, max(1e-3, look_z))

        self._publish_reference(u_d, v_d, w_d, 0.0, theta_d, psi)

        err = math.sqrt(ex_n * ex_n + ey_e * ey_e + ez * ez)
        if err <= tol:
            self._publish_status(
                f"station_keeping_3d: holding "
                f"pos=({self._x:.2f},{self._y:.2f},{self._z:.2f}) "
                f"target=({tx:.2f},{ty:.2f},{tz:.2f}) err={err:.3f}m"
            )
        else:
            self._publish_status(
                f"station_keeping_3d: approaching "
                f"pos=({self._x:.2f},{self._y:.2f},{self._z:.2f}) "
                f"target=({tx:.2f},{ty:.2f},{tz:.2f}) err={err:.3f}m "
                f"u_d={u_d:.2f} v_d={v_d:.2f} w_d={w_d:.2f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StationKeepingMission3d()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
