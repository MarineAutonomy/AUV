"""Republish sensor_msgs/Imu from ENU (ROS) into NED for the AUV EKF.

Subscribes:  /imu/data          (ENU world + ROS FLU body, e.g. SBG use_enu:=true)
Publishes:   /imu/data/ned      (NED world + NED/FRD-style body axes)

World vectors / attitude use the fixed ENU→NED map:
  [n, e, d]^T = [[0,1,0],[1,0,0],[0,0,-1]] [e, n, u]^T

Body rates/accels from SBG ENU mode are FLU (y,z flipped vs native NED body);
undo that with [x, -y, -z] so the KF sees NED-consistent body axes.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ENU → NED for free vectors / rotating the attitude into NED.
R_NED_ENU = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)

# FLU (ROS / SBG use_enu body) → FRD/NED-native body used by the KF.
M_FLU_TO_NED_BODY = np.diag([1.0, -1.0, -1.0])


def _quat_to_rotm(w: float, x: float, y: float, z: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _rotm_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """Shepperd's method → (w, x, y, z)."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = float(np.trace(R))
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return w / n, x / n, y / n, z / n


def _transform_cov3(cov9, M: np.ndarray) -> list[float]:
    """Row-major 3x3 covariance → M Cov M^T."""
    C = np.asarray(cov9, dtype=float).reshape(3, 3)
    if not np.isfinite(C).all() or np.allclose(C, 0.0):
        return [float(v) for v in cov9]
    Ct = M @ C @ M.T
    return Ct.reshape(9).tolist()


def imu_enu_to_ned(msg: Imu) -> Imu:
    out = Imu()
    out.header = msg.header
    out.header.frame_id = "NED"

    # Attitude: body(FLU)→ENU  ⇒  body(FRD)→NED
    q = msg.orientation
    R_enu_flu = _quat_to_rotm(q.w, q.x, q.y, q.z)
    R_ned_frd = R_NED_ENU @ R_enu_flu @ M_FLU_TO_NED_BODY
    w, x, y, z = _rotm_to_quat(R_ned_frd)
    out.orientation.w = w
    out.orientation.x = x
    out.orientation.y = y
    out.orientation.z = z
    out.orientation_covariance = _transform_cov3(msg.orientation_covariance, R_NED_ENU)

    # Body rates / specific force: FLU → NED body
    gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
    ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
    g_ned = M_FLU_TO_NED_BODY @ np.array([gx, gy, gz], dtype=float)
    a_ned = M_FLU_TO_NED_BODY @ np.array([ax, ay, az], dtype=float)
    out.angular_velocity.x = float(g_ned[0])
    out.angular_velocity.y = float(g_ned[1])
    out.angular_velocity.z = float(g_ned[2])
    out.linear_acceleration.x = float(a_ned[0])
    out.linear_acceleration.y = float(a_ned[1])
    out.linear_acceleration.z = float(a_ned[2])
    out.angular_velocity_covariance = _transform_cov3(msg.angular_velocity_covariance, M_FLU_TO_NED_BODY)
    out.linear_acceleration_covariance = _transform_cov3(
        msg.linear_acceleration_covariance, M_FLU_TO_NED_BODY
    )
    return out


class ImuEnuToNedNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_enu_to_ned")
        self.declare_parameter("input_topic", "/imu/data")
        self.declare_parameter("output_topic", "/imu/data/ned")

        inp = str(self.get_parameter("input_topic").value)
        out = str(self.get_parameter("output_topic").value)

        self._pub = self.create_publisher(Imu, out, 10)
        self._sub = self.create_subscription(Imu, inp, self._on_imu, 10)
        self.get_logger().info(f"imu_enu_to_ned: {inp} → {out} (ENU/FLU → NED)")

    def _on_imu(self, msg: Imu) -> None:
        self._pub.publish(imu_enu_to_ned(msg))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuEnuToNedNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
