"""Depth-hold / hover mission for 3D AUV guidance."""

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
    """Extract yaw (psi) from orientation quaternion (NED-friendly ZYX)."""
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DepthControlMission3d(Node):
    """
    Hover / depth-hold: regulate NED z to target_depth (positive down = depth).

    Publishes [u_d, v_d, w_d, phi_d, theta_d, psi_d] on /guidance/reference_3d
    with u=v=0, psi held at the heading seen when the node starts (or latest odom).
    """

    def __init__(self) -> None:
        super().__init__("depth_control_mission_3d")

        self.declare_parameter("odom_topic", "navigation/odometry")
        self.declare_parameter("reference_topic", "/guidance/reference_3d")
        self.declare_parameter("status_topic", "/mission/status")

        self.declare_parameter("target_depth", 1.0)
        self.declare_parameter("w_gain", 0.7)
        self.declare_parameter("w_max", 0.25)
        self.declare_parameter("lookahead_z", 0.7)
        self.declare_parameter("depth_tolerance", 0.08)

        odom_topic = str(self.get_parameter("odom_topic").value)
        reference_topic = str(self.get_parameter("reference_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self._z: Optional[float] = None
        self._psi_hold: Optional[float] = None

        self._ref_pub = self.create_publisher(Float64MultiArray, reference_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._timer = self.create_timer(0.1, self._on_timer)

        self.get_logger().info(
            f"depth_control_mission_3d ready — target_depth="
            f"{float(self.get_parameter('target_depth').value):.3f} m (NED z)"
        )

    def _on_odom(self, msg: Odometry) -> None:
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
        if self._z is None or self._psi_hold is None:
            return

        target = float(self.get_parameter("target_depth").value)
        w_gain = float(self.get_parameter("w_gain").value)
        w_max = float(self.get_parameter("w_max").value)
        look_z = float(self.get_parameter("lookahead_z").value)
        tol = float(self.get_parameter("depth_tolerance").value)

        # NED: z positive down. error > 0 ⇒ too deep ⇒ command negative heave (up).
        ez = self._z - target
        w_d = clamp(-w_gain * ez, -w_max, w_max)
        theta_d = math.atan2(-ez, max(1e-3, look_z))
        psi_d = float(self._psi_hold)

        self._publish_reference(0.0, 0.0, w_d, 0.0, theta_d, psi_d)

        if abs(ez) <= tol:
            self._publish_status(
                f"depth_control_3d: holding depth={self._z:.2f}m target={target:.2f}m err={ez:.3f}m"
            )
        else:
            self._publish_status(
                f"depth_control_3d: approaching depth={self._z:.2f}m target={target:.2f}m "
                f"err={ez:.3f}m w_d={w_d:.3f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthControlMission3d()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
