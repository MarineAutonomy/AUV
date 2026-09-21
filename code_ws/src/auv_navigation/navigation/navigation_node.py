#!/usr/bin/env python3
"""Kinematic EKF for the AUV: same topics/sensors as auv_navigation; filter core in navigation.*."""

import threading

import numpy as np
import rclpy
from dvl_msgs.msg import DVL, DVLDR
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from sensor_msgs.msg import Imu, JointState, Range
from std_msgs.msg import Float32, Float64MultiArray

from navigation.ekf_nav import NavEKF
from navigation.nav_kinematics import eul_to_rotm, eul_to_quat, quat_to_eul, quat_to_rotm
from navigation.nav_observations import (
    jacobian_depth,
    jacobian_dr_position,
    jacobian_dvl,
    jacobian_encoders,
    jacobian_imu_numerical,
    jacobian_range,
    predict_depth,
    predict_dr_position,
    predict_dvl_body,
    predict_imu,
    predict_range,
)
from navigation.read_config import read_vessel_data


def calculate_threshold(dt, n_actuators):
    th = np.full(15 + n_actuators, np.inf)
    omg_der_var = 0.1 ** 2
    acc_der_var = 0.5 ** 2
    cov = np.zeros((6, 6))
    cov[0:3, 0:3] = omg_der_var * np.eye(3)
    cov[3:6, 3:6] = acc_der_var * np.eye(3)
    return th, cov


def _joint_values_ordered(msg: JointState, names, field: str):
    arr = getattr(msg, field)
    return np.array([arr[msg.name.index(n)] for n in names])


def _imu_covariance_9x9(msg: Imu, sensor: dict) -> np.ndarray:
    if sensor.get("use_custom_covariance"):
        cc = sensor["custom_covariance"]
        R = np.zeros((9, 9))
        R[0:3, 0:3] = np.array(cc["orientation_covariance"]).reshape(3, 3)
        R[3:6, 3:6] = np.array(cc["angular_velocity_covariance"]).reshape(3, 3)
        R[6:9, 6:9] = np.array(cc["linear_acceleration_covariance"]).reshape(3, 3)
        return R
    R = np.zeros((9, 9))
    for block, src, key, default_v in [
        (slice(0, 3), msg.orientation_covariance, "default_orientation_variance", 0.05),
        (slice(3, 6), msg.angular_velocity_covariance, "default_gyro_variance", 0.02),
        (slice(6, 9), msg.linear_acceleration_covariance, "default_accel_variance", 0.1),
    ]:
        arr = np.array(src)
        if len(arr) >= 9 and arr[0] >= 0:
            R[block, block] = arr.reshape(3, 3)
        else:
            v = float(sensor.get(key, default_v))
            R[block, block] = np.eye(3) * v
    return R


class NavigationFilterNode(Node):
    def __init__(self, vessel, ekf: NavEKF, th=None, apply_prefix=False, vessel_file=None):
        super().__init__("navigation")

        self.vessel = vessel
        try:
            assert "agents" in self.vessel and len(self.vessel["agents"]) > 0
            assert "id" in self.vessel["agents"][0]
            assert "name" in self.vessel["agents"][0]
            assert "sensors" in self.vessel["agents"][0]
            assert len(self.vessel["agents"][0]["sensors"]) > 0
        except AssertionError as e:
            self.get_logger().error(f"Vessel configuration error: {e}")
            raise

        self.vessel_id = vessel["agents"][0]["id"]
        self.vessel_name = vessel["agents"][0]["name"]

        if apply_prefix:
            try:
                self.topic_prefix = vessel["agents"][0]["prefix"]
            except Exception:
                self.topic_prefix = None
            if self.topic_prefix is None or str(self.topic_prefix).strip() == "":
                self.topic_prefix = f"{self.vessel_name}_{self.vessel_id:02d}"
            self.get_logger().info(f"Topic prefix: {self.topic_prefix}")
        else:
            self.topic_prefix = ""

        self.ekf = ekf
        self._ekf_lock = threading.Lock()
        self.first_imu_flag = True
        self.first_pos_flag = True
        self.n_state = ekf.n_states

        self._imu_init_yaw = None
        self._imu_latest_yaw = None
        self._dvl_dr_R_to_ned = None
        self._dvl_dr_origin = None
        self._dvl_dr_initialized = False
        self._dvl_dr_init_time = None

        self._sbg_latest_quat = None
        self._sbg_quat_lock = threading.Lock()
        self._imu_yaw_lock = threading.Lock()

        g = vessel.get("mahalanobis_gate")
        if g is None or g is False or (isinstance(g, str) and g.strip().lower() in ("", "none", "false")):
            self.mahalanobis_gate = None
        else:
            self.mahalanobis_gate = float(g)

        pool_depth = vessel.get("pool_depth")
        dvl_depth = vessel.get("dvl_depth_from_surface")

        if pool_depth is not None and dvl_depth is not None:
            self._dvl_surface_offset = float(dvl_depth)
            self.seabed_z_ned = float(pool_depth) - self._dvl_surface_offset
            self.get_logger().info(
                f"Pool depth={pool_depth}m, DVL submersion={dvl_depth}m "
                f"→ seabed_z_ned={self.seabed_z_ned:.3f}m, "
                f"depth sensor offset={-self._dvl_surface_offset:.3f}m"
            )
        elif vessel.get("seabed_z_ned") is not None:
            self.seabed_z_ned = float(vessel["seabed_z_ned"])
            self._dvl_surface_offset = 0.0
            self.get_logger().info(
                f"Using legacy seabed_z_ned={self.seabed_z_ned}m (no DVL surface offset)"
            )
        else:
            self.seabed_z_ned = None
            self._dvl_surface_offset = 0.0

        self._has_ping_range = any(
            s.get("sensor_type") == "PING_RANGE" for s in self.vessel["agents"][0]["sensors"]
        )
        if self._has_ping_range and self.seabed_z_ned is None:
            raise ValueError(
                "vessel_data.yml: pool_depth + dvl_depth_from_surface (or legacy seabed_z_ned) "
                "required when using sensor_type PING_RANGE."
            )

        for sensor in self.vessel["agents"][0]["sensors"]:
            st = sensor["sensor_type"]
            topic = self._sensor_topic(sensor, apply_prefix)

            if st == "IMU":

                def imu_cb(msg, s=sensor):
                    self.imu_callback(msg, s)

                self.create_subscription(Imu, topic, imu_cb, 10)

            elif st == "IMU_SBG":
                quat_topic = sensor.get("quat_topic", "/sbg/ekf_quat")
                if apply_prefix:
                    if quat_topic.startswith("/"):
                        quat_topic = f"/{self.topic_prefix}{quat_topic}"
                    else:
                        quat_topic = f"/{self.topic_prefix}/{quat_topic}"

                def sbg_quat_cb(msg):
                    with self._sbg_quat_lock:
                        self._sbg_latest_quat = msg

                self.create_subscription(SbgEkfQuat, quat_topic, sbg_quat_cb, 10)

                def sbg_imu_cb(msg, s=sensor):
                    self.sbg_imu_callback(msg, s)

                self.create_subscription(SbgImuData, topic, sbg_imu_cb, 10)
                self.get_logger().info(
                    f"IMU_SBG: imu_data={topic}, ekf_quat={quat_topic}"
                )

            elif st == "DVL":

                def dvl_msg_cb(msg, s=sensor):
                    self.dvl_a50_callback(msg, s)

                # DVL drivers commonly publish BEST_EFFORT; match QoS to avoid dropping all messages.
                self.create_subscription(DVL, topic, dvl_msg_cb, qos_profile_sensor_data)

            elif st == "DVL_DR":

                def dr_cb(msg, s=sensor):
                    self.dvl_dr_callback(msg, s)

                # DVL DR is also typically BEST_EFFORT.
                self.create_subscription(DVLDR, topic, dr_cb, qos_profile_sensor_data)

            elif st == "encoders":
                field = sensor.get("value_field", "position")

                def enc_cb(msg, s=sensor, f=field):
                    self.encoders_callback(msg, s, f)

                self.create_subscription(JointState, topic, enc_cb, 10)

            elif st == "ARDUINO_DEPTH":

                def ad_cb(msg, s=sensor):
                    self.arduino_depth_callback(msg, s)

                self.create_subscription(Float32, topic, ad_cb, 10)

            elif st == "PING_RANGE":

                def ping_cb(msg, s=sensor):
                    self.ping_range_callback(msg, s)

                self.create_subscription(Range, topic, ping_cb, 10)

            else:
                self.get_logger().warn(f"Unknown sensor_type '{st}', skipping.")

        odom_name = vessel["agents"][0].get("odometry_topic", "navigation/odometry")
        state_name = vessel["agents"][0].get("state_topic", "navigation/state")
        if apply_prefix:
            self.odom_topic = f"/{self.topic_prefix}/{odom_name}"
            self.state_topic = f"/{self.topic_prefix}/{state_name}"
        else:
            self.odom_topic = odom_name if odom_name.startswith("/") else f"/{odom_name}"
            self.state_topic = state_name if state_name.startswith("/") else f"/{state_name}"

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.state_pub = self.create_publisher(Float64MultiArray, self.state_topic, 10)

        self.create_timer(float(self.vessel["time_step"]), self.update_odometry)

        self.dt = float(self.vessel["time_step"])
        self.th = th
        if vessel_file:
            self.get_logger().info(f"Vessel data file: {vessel_file}")
        self.get_logger().info(f"Publishing odometry: {self.odom_topic}")

    def _sensor_topic(self, sensor, apply_prefix):
        if "sensor_topic" not in sensor:
            defaults = {
                "IMU": "/imu/data/ned",
                "IMU_SBG": "/sbg/imu_data",
                "DVL": "/dvl/data",
                "DVL_DR": "/dvl/position",
                "encoders": "/actuator_feedback",
                "ARDUINO_DEPTH": "/auv/depth",
                "PING_RANGE": "/ping1d/range",
            }
            rel = defaults.get(sensor["sensor_type"], "/sensor")
        else:
            rel = sensor["sensor_topic"]
        if not apply_prefix:
            return rel if rel.startswith("/") else f"/{rel}"
        if rel.startswith("/"):
            return f"/{self.topic_prefix}{rel}"
        return f"/{self.topic_prefix}/{rel}"

    def encoders_callback(self, msg, sensor, field: str):
        names = sensor["actuator_names"]
        y_encoders = _joint_values_ordered(msg, names, field)[:, np.newaxis]
        var = sensor.get("default_variance", [0.01] * len(names))
        R_encoders = np.diag(var)
        n_act = len(names)
        H = jacobian_encoders(self.n_state, n_act)
        with self._ekf_lock:
            xv = self.ekf.x.flatten()
            h_x = xv[15 : 15 + n_act].reshape(-1, 1)
            self.ekf.update(
                y_encoders,
                H,
                R_encoders,
                h_x,
                threshold=self.th,
                imu_ssa=False,
                mahalanobis_gate=self.mahalanobis_gate,
            )

    def imu_callback(self, msg, sensor):
        imu_quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        imu_eul = quat_to_eul(imu_quat)
        with self._imu_yaw_lock:
            self._imu_latest_yaw = float(imu_eul[2])
        imu_acc = np.array(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        )
        imu_omg = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )
        imu_acc = imu_acc - quat_to_rotm(imu_quat).T @ np.array([0, 0, -9.81])
        y_imu = np.concatenate((imu_eul, imu_omg, imu_acc))[:, np.newaxis]

        r_bs_b = np.array(sensor["sensor_location"])
        Theta_bs = np.array(sensor["sensor_orientation"])
        R_imu = _imu_covariance_9x9(msg, sensor)
        with self._ekf_lock:
            xv = self.ekf.x.flatten()
            H = jacobian_imu_numerical(xv, r_bs_b, Theta_bs, self.n_state)

            if self.first_pos_flag or self.first_imu_flag:
                self.ekf.x[3:6] = y_imu[0:3]
                self.ekf.x[9:12] = y_imu[3:6]
                self.ekf.x[12:15] = y_imu[6:9]
                if self.first_imu_flag:
                    self._imu_init_yaw = float(imu_eul[2])
                    self.get_logger().info(
                        f"IMU init yaw (magnetic NED): {np.degrees(self._imu_init_yaw):.1f} deg"
                    )
                    self.first_imu_flag = False
            else:
                h_x = predict_imu(xv, r_bs_b, Theta_bs).reshape(-1, 1)
                self.ekf.update(
                    y_imu,
                    H,
                    R_imu,
                    h_x,
                    threshold=self.th,
                    imu_ssa=True,
                    mahalanobis_gate=self.mahalanobis_gate,
                )

    def sbg_imu_callback(self, msg: SbgImuData, sensor: dict):
        with self._sbg_quat_lock:
            quat_msg = self._sbg_latest_quat
        if quat_msg is None:
            return

        q = quat_msg.quaternion
        imu_quat = np.array([q.w, q.x, q.y, q.z])
        imu_eul = quat_to_eul(imu_quat)

        imu_omg = np.array([msg.gyro.x, msg.gyro.y, msg.gyro.z])
        imu_acc = np.array([msg.accel.x, msg.accel.y, msg.accel.z])
        imu_acc = imu_acc - quat_to_rotm(imu_quat).T @ np.array([0, 0, -9.81])

        y_imu = np.concatenate((imu_eul, imu_omg, imu_acc))[:, np.newaxis]

        r_bs_b = np.array(sensor["sensor_location"])
        Theta_bs = np.array(sensor["sensor_orientation"])
        R_imu = self._sbg_imu_covariance(quat_msg, sensor)

        with self._ekf_lock:
            xv = self.ekf.x.flatten()
            H = jacobian_imu_numerical(xv, r_bs_b, Theta_bs, self.n_state)

            if self.first_pos_flag or self.first_imu_flag:
                self.ekf.x[3:6] = y_imu[0:3]
                self.ekf.x[9:12] = y_imu[3:6]
                self.ekf.x[12:15] = y_imu[6:9]
                if self.first_imu_flag:
                    self._imu_init_yaw = float(imu_eul[2])
                    self.get_logger().info(
                        f"SBG IMU init yaw (NED): {np.degrees(self._imu_init_yaw):.1f} deg"
                    )
                    self.first_imu_flag = False
            else:
                h_x = predict_imu(xv, r_bs_b, Theta_bs).reshape(-1, 1)
                self.ekf.update(
                    y_imu,
                    H,
                    R_imu,
                    h_x,
                    threshold=self.th,
                    imu_ssa=True,
                    mahalanobis_gate=self.mahalanobis_gate,
                )

    @staticmethod
    def _sbg_imu_covariance(quat_msg: SbgEkfQuat, sensor: dict) -> np.ndarray:
        if sensor.get("use_custom_covariance"):
            cc = sensor["custom_covariance"]
            R = np.zeros((9, 9))
            R[0:3, 0:3] = np.array(cc["orientation_covariance"]).reshape(3, 3)
            R[3:6, 3:6] = np.array(cc["angular_velocity_covariance"]).reshape(3, 3)
            R[6:9, 6:9] = np.array(cc["linear_acceleration_covariance"]).reshape(3, 3)
            return R
        R = np.zeros((9, 9))
        acc = quat_msg.accuracy
        ori_var = np.array([acc.x, acc.y, acc.z]) ** 2
        R[0:3, 0:3] = np.diag(ori_var)
        gyro_var = float(sensor.get("default_gyro_variance", 0.02))
        R[3:6, 3:6] = np.eye(3) * gyro_var
        accel_var = float(sensor.get("default_accel_variance", 0.1))
        R[6:9, 6:9] = np.eye(3) * accel_var
        return R

    def dvl_a50_callback(self, msg: DVL, sensor: dict):
        if not msg.velocity_valid:
            return
        y = np.array([[msg.velocity.x], [msg.velocity.y], [msg.velocity.z]])
        r_bs_b = np.array(sensor["sensor_location"])
        Theta_bs = np.array(sensor["sensor_orientation"])
        if len(msg.covariance) >= 9:
            R = np.array(msg.covariance, dtype=float).reshape(3, 3)
        elif sensor.get("use_custom_covariance"):
            R = np.array(sensor["custom_covariance"]["velocity_covariance"]).reshape(3, 3)
        else:
            v = float(sensor.get("default_velocity_variance", 0.05))
            fom_scale = float(sensor.get("fom_variance_scale", 0.0))
            if fom_scale > 0.0 and msg.fom > 0.0:
                v = max(v, fom_scale * float(msg.fom) ** 2)
            R = np.eye(3) * v
        with self._ekf_lock:
            xv = self.ekf.x.flatten()
            H = jacobian_dvl(xv, r_bs_b, Theta_bs, self.n_state)
            h_x = predict_dvl_body(xv, r_bs_b, Theta_bs).reshape(-1, 1)
            self.ekf.update(
                y,
                H,
                R,
                h_x,
                threshold=self.th,
                imu_ssa=False,
                mahalanobis_gate=self.mahalanobis_gate,
            )

    def _dvl_dr_to_ned(self, raw: np.ndarray) -> np.ndarray:
        """Rotate DVL DR position from the DVL's power-on frame into the IMU's magnetic NED frame."""
        if self._dvl_dr_R_to_ned is None or self._dvl_dr_origin is None:
            return raw
        local = raw - self._dvl_dr_origin
        return self._dvl_dr_R_to_ned @ local

    def dvl_dr_callback(self, msg: DVLDR, sensor: dict):
        y_raw = np.array([msg.position.x, msg.position.y, msg.position.z])
        r_bs_b = np.array(sensor["sensor_location"])
        Theta_bs = np.array(sensor["sensor_orientation"])
        std = float(msg.pos_std)
        base = max(std**2, float(sensor.get("min_position_variance", 1e-4)))
        scale = float(sensor.get("position_variance_scale", 1.0))
        scale = max(scale, 1e-6)
        drift_rate = float(sensor.get("dr_drift_rate", 0.0))
        if self._dvl_dr_init_time is not None and drift_rate > 0:
            elapsed = (self.get_clock().now().nanoseconds - self._dvl_dr_init_time) * 1e-9
            base += drift_rate * elapsed
        R = np.eye(3) * (base * scale)
        if sensor.get("use_custom_covariance"):
            R = np.array(sensor["custom_covariance"]["position_covariance"]).reshape(3, 3)

        with self._ekf_lock:
            if not self._dvl_dr_initialized:
                dvl_yaw = float(msg.yaw) * (np.pi / 180.0)
                imu_yaw_now = None
                with self._sbg_quat_lock:
                    quat_now = self._sbg_latest_quat
                if quat_now is not None:
                    q = quat_now.quaternion
                    sbg_quat = np.array([q.w, q.x, q.y, q.z])
                    imu_yaw_now = float(quat_to_eul(sbg_quat)[2])
                else:
                    with self._imu_yaw_lock:
                        imu_yaw_now = self._imu_latest_yaw
                if imu_yaw_now is None:
                    self.get_logger().warn(
                        "DVL DR received before IMU yaw (/imu/data/ned); deferring DR init."
                    )
                    return
                self._dvl_dr_origin = y_raw.copy()
                delta_psi = float(imu_yaw_now) - dvl_yaw
                c, s = np.cos(delta_psi), np.sin(delta_psi)
                self._dvl_dr_R_to_ned = np.array([
                    [ c, -s,  0.0],
                    [ s,  c,  0.0],
                    [0.0, 0.0, 1.0],
                ])
                self.get_logger().info(
                    f"DVL DR→NED: IMU yaw={np.degrees(imu_yaw_now):.1f}°, "
                    f"DVL yaw={np.degrees(dvl_yaw):.1f}°, "
                    f"delta={np.degrees(delta_psi):.1f}°"
                )
                self._dvl_dr_initialized = True
                self._dvl_dr_init_time = self.get_clock().now().nanoseconds
                y_ned = self._dvl_dr_to_ned(y_raw)
                self.ekf.x[0:3] = y_ned.reshape(3, 1)
                if self.first_pos_flag:
                    self.first_pos_flag = False
            else:
                y_ned = self._dvl_dr_to_ned(y_raw)
                y = y_ned.reshape(3, 1)
                if self._dvl_dr_R_to_ned is not None:
                    Rr = self._dvl_dr_R_to_ned
                    R = Rr @ R @ Rr.T
                xv = self.ekf.x.flatten()
                H = jacobian_dr_position(xv, r_bs_b, Theta_bs, self.n_state)
                h_x = predict_dr_position(xv, r_bs_b, Theta_bs).reshape(-1, 1)
                self.ekf.update(
                    y,
                    H,
                    R,
                    h_x,
                    threshold=self.th,
                    imu_ssa=False,
                    mahalanobis_gate=self.mahalanobis_gate,
                )

    def _depth_measurement_update(self, depth_metres: float, sensor: dict):
        if not np.isfinite(depth_metres):
            return
        sign_scale = float(sensor.get("sign_scale", 1.0))
        z_off = float(sensor.get("z_offset", 0.0))
        y = np.array([[sign_scale * depth_metres + z_off - self._dvl_surface_offset]])
        r_bs_b = np.array(sensor["sensor_location"])
        Theta_bs = np.array(sensor["sensor_orientation"])
        var = float(sensor.get("default_variance", 0.05))
        R = np.array([[var]])
        with self._ekf_lock:
            xv = self.ekf.x.flatten()

            if self.first_pos_flag or self.first_imu_flag:
                self.ekf.x[2, 0] = float(y[0, 0])
                if self.first_pos_flag:
                    self.first_pos_flag = False
            else:
                H = jacobian_depth(xv, r_bs_b, Theta_bs, self.n_state)
                h_x = np.array([[predict_depth(xv, r_bs_b, Theta_bs)]])
                self.ekf.update(
                    y,
                    H,
                    R,
                    h_x,
                    threshold=self.th,
                    imu_ssa=False,
                    mahalanobis_gate=self.mahalanobis_gate,
                )

    def arduino_depth_callback(self, msg: Float32, sensor: dict):
        d = float(msg.data)
        if not np.isfinite(d):
            return
        self._depth_measurement_update(d, sensor)

    def ping_range_callback(self, msg: Range, sensor: dict):
        r = float(msg.range)
        if not np.isfinite(r) or not (float(msg.min_range) <= r <= float(msg.max_range)):
            return
        z_sb = float(sensor.get("seabed_z_ned", self.seabed_z_ned))
        r_bs_b = np.array(sensor["sensor_location"])
        Theta_bs = np.array(sensor["sensor_orientation"])
        y = np.array([[r]])
        var = float(sensor.get("default_variance", 0.04))
        R = np.array([[var]])
        with self._ekf_lock:
            xv = self.ekf.x.flatten()

            if self.first_pos_flag or self.first_imu_flag:
                R_nb = eul_to_rotm(
                    np.array([self.ekf.x[3, 0], self.ekf.x[4, 0], self.ekf.x[5, 0]])
                )
                z_sensor = z_sb - r
                z_body = z_sensor - (R_nb @ r_bs_b)[2]
                self.ekf.x[2, 0] = float(z_body)
                if self.first_pos_flag:
                    self.first_pos_flag = False
            else:
                H = jacobian_range(xv, z_sb, r_bs_b, Theta_bs, self.n_state)
                h_x = np.array([[predict_range(xv, z_sb, r_bs_b, Theta_bs)]])
                self.ekf.update(
                    y,
                    H,
                    R,
                    h_x,
                    threshold=self.th,
                    imu_ssa=False,
                    mahalanobis_gate=self.mahalanobis_gate,
                )

    def update_odometry(self):
        u = np.zeros(self.ekf.n_inp)
        with self._ekf_lock:
            self.ekf.predict(u=u, threshold=self.th)

            x = self.ekf.x.flatten().copy()
            P = self.ekf.P.copy()

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "NED"
        odom_msg.child_frame_id = "BODY"
        odom_msg.pose.pose.position.x = float(x[0])
        odom_msg.pose.pose.position.y = float(x[1])
        odom_msg.pose.pose.position.z = float(x[2])
        quat = eul_to_quat(x[3:6])
        odom_msg.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        pose_cov = np.zeros(36).reshape(6, 6)
        pose_cov[0:3, 0:3] = P[0:3, 0:3]
        pose_cov[3:6, 3:6] = P[3:6, 3:6]
        odom_msg.pose.covariance = pose_cov.flatten()
        odom_msg.twist.twist.linear.x = float(x[6])
        odom_msg.twist.twist.linear.y = float(x[7])
        odom_msg.twist.twist.linear.z = float(x[8])
        odom_msg.twist.twist.angular.x = float(x[9])
        odom_msg.twist.twist.angular.y = float(x[10])
        odom_msg.twist.twist.angular.z = float(x[11])
        twist_cov = np.zeros(36).reshape(6, 6)
        twist_cov[0:3, 0:3] = P[6:9, 6:9]
        twist_cov[3:6, 3:6] = P[9:12, 9:12]
        odom_msg.twist.covariance = twist_cov.flatten()
        self.odom_pub.publish(odom_msg)

        state_msg = Float64MultiArray()
        state_msg.data = x.tolist()
        self.state_pub.publish(state_msg)


def main():
    rclpy.init()
    loader = rclpy.create_node("nav_config_loader")
    loader.declare_parameter("vessel_data_file", "")
    vf = loader.get_parameter("vessel_data_file").get_parameter_value().string_value
    loader.destroy_node()

    path = vf.strip() if vf else None
    vessel = read_vessel_data(path)

    dt = float(vessel["time_step"])

    try:
        thrusters = vessel["agents"][0]["thrusters"]
        if thrusters in (None, "None", "none"):
            thrusters = None
    except Exception:
        thrusters = None
    try:
        control_surfaces = vessel["agents"][0].get("control_surfaces")
        if control_surfaces in (None, "None", "none"):
            control_surfaces = None
    except Exception:
        control_surfaces = None

    n_thrusters = len(thrusters) if thrusters else 0
    n_surfaces = len(control_surfaces) if control_surfaces else 0
    n_actuators = n_thrusters + n_surfaces

    th, cov = calculate_threshold(dt, n_actuators)
    ekf = NavEKF(dt, n_states=15 + n_actuators, n_inp=n_actuators, pro_noise_cov=cov)

    executor = MultiThreadedExecutor()
    apply_prefix = vessel["agents"][0].get("apply_prefix", False)
    nav = NavigationFilterNode(
        vessel,
        ekf,
        th=th,
        apply_prefix=apply_prefix,
        vessel_file=path,
    )
    executor.add_node(nav)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        nav.destroy_node()
        # Can already be shutdown when SIGINT propagates via rclpy internals.
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
