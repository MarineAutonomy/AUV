#!/usr/bin/env python3
"""
auv_bridge.py
=============
ROS2 node that bridges Arduino serial communication:

  SUBSCRIBES:
    /auv/thruster_cmd  [std_msgs/Int32MultiArray]
        → 8 PWM values [t1..t8] (1100–1900, 1500=neutral)
        → serial: "v0,v1,v2,v3,v4,v5,v6,v7\\n"

    /auv/light_cmd     [std_msgs/Int32MultiArray]
        → 2 PWM values [lightfl, lightbl] (1100–1900, 1100=off, 1900=full)
        → serial: "L,<fl>,<bl>\\n"
          lightfl = pin 5 (light1), lightbl = pin 4 (light2)

  PUBLISHES:
    /auv/pressure      [std_msgs/Float32]   — mbar
    /auv/temperature   [std_msgs/Float32]   — °C
    /auv/depth         [std_msgs/Float32]   — metres
    /auv/sensor_raw    [std_msgs/String]    — raw CSV line for debug

Usage:
  ros2 run arduino_ps arduino_ps --ros-args \\
      -p port:=/dev/arduino_mega -p baud:=115200

Publish thruster command from terminal (example):
  ros2 topic pub /auv/thruster_cmd std_msgs/Int32MultiArray \\
      "{data: [1500,1500,1500,1500,1500,1500,1500,1500]}"

Publish light brightness (example — both half):
  ros2 topic pub /auv/light_cmd std_msgs/Int32MultiArray \\
      "{data: [1500,1500]}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32MultiArray

import serial
import threading
import time


class AUVBridge(Node):

    def __init__(self):
        super().__init__('auv_bridge')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('port', '/dev/arduino_mega')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_period', 0.1)   # 10 Hz

        port        = self.get_parameter('port').get_parameter_value().string_value
        baud        = self.get_parameter('baud').get_parameter_value().integer_value
        send_period = self.get_parameter('send_period').get_parameter_value().double_value
        # ──────────────────────────────────────────────────────

        # ── Publishers ────────────────────────────────────────
        self.pub_pressure    = self.create_publisher(Float32,         '/auv/pressure',     10)
        self.pub_temperature = self.create_publisher(Float32,         '/auv/temperature',  10)
        self.pub_depth       = self.create_publisher(Float32,         '/auv/depth',        10)
        self.pub_raw         = self.create_publisher(String,          '/auv/sensor_raw',   10)
        # ──────────────────────────────────────────────────────

        # ── Subscribers ───────────────────────────────────────
        self.sub_thruster = self.create_subscription(
            Int32MultiArray,
            '/auv/thruster_cmd',
            self.thruster_callback,
            10
        )
        self.sub_light = self.create_subscription(
            Int32MultiArray,
            '/auv/light_cmd',
            self.light_callback,
            10
        )
        # ──────────────────────────────────────────────────────

        # ── Command state ─────────────────────────────────────
        self.thruster_values = [1500] * 8
        self.thruster_lock   = threading.Lock()

        # Lumen lights: [lightfl, lightbl] — off until commanded
        self.light_values = [1100, 1100]
        self.light_lock   = threading.Lock()
        self.light_dirty  = True   # send once after boot
        # ──────────────────────────────────────────────────────

        # ── Serial port ───────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0, write_timeout=2.0)
            self.get_logger().info(f'Opened serial port {port} @ {baud} baud')
            time.sleep(3.0)                  # wait for Arduino to boot
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {port}: {e}')
            raise SystemExit(1)
        # ──────────────────────────────────────────────────────

        # ── Background serial reader thread ───────────────────
        self.stop_flag = threading.Event()
        self.reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True
        )
        self.reader_thread.start()
        # ──────────────────────────────────────────────────────

        # ── Timer: thrusters every period; lights when dirty ──
        self.create_timer(send_period, self._send_commands)
        # ──────────────────────────────────────────────────────

        self.get_logger().info('AUV Bridge node started.')
        self.get_logger().info('Subscribe to /auv/thruster_cmd (8 PWM) and /auv/light_cmd (2 PWM).')
        self.get_logger().info('Sensor data on /auv/pressure, /auv/temperature, /auv/depth')

    # ── Thruster subscriber callback ──────────────────────────
    def thruster_callback(self, msg: Int32MultiArray):
        """Receives 8 PWM values and stores them for next serial send."""
        if len(msg.data) != 8:
            self.get_logger().warn(
                f'Expected 8 thruster values, got {len(msg.data)} — ignoring'
            )
            return

        clamped = [max(1100, min(1900, int(v))) for v in msg.data]

        with self.thruster_lock:
            self.thruster_values = clamped

        self.get_logger().debug(f'Thruster cmd received: {clamped}')

    # ── Light subscriber callback ─────────────────────────────
    def light_callback(self, msg: Int32MultiArray):
        """Receives 2 Lumen PWM values [lightfl, lightbl] (1100–1900)."""
        if len(msg.data) != 2:
            self.get_logger().warn(
                f'Expected 2 light values [fl, bl], got {len(msg.data)} — ignoring'
            )
            return

        clamped = [max(1100, min(1900, int(v))) for v in msg.data]

        with self.light_lock:
            self.light_values = clamped
            self.light_dirty = True

        self.get_logger().debug(f'Light cmd received: fl={clamped[0]} bl={clamped[1]}')

    # ── Timer: thrusters always; lights when changed ──────────
    def _send_commands(self):
        with self.thruster_lock:
            thrusters = list(self.thruster_values)

        thruster_line = ",".join(str(v) for v in thrusters) + "\n"
        try:
            self.ser.write(thruster_line.encode("ascii"))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error (thrusters): {e}')
            return

        light_line = None
        with self.light_lock:
            if self.light_dirty:
                fl, bl = self.light_values
                light_line = f"L,{fl},{bl}\n"
                self.light_dirty = False

        if light_line is not None:
            try:
                self.ser.write(light_line.encode("ascii"))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error (lights): {e}')
                with self.light_lock:
                    self.light_dirty = True
                return

        try:
            self.ser.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial flush error: {e}')

    # ── Background reader: parse sensor data from Arduino ─────
    def _serial_reader(self):
        while not self.stop_flag.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Publish raw line always (useful for debug)
                raw_msg = String()
                raw_msg.data = line
                self.pub_raw.publish(raw_msg)

                if line.startswith("S,"):
                    # ── Parse:  S,<pressure>,<temp>,<depth> ──
                    parts = line.split(",")
                    if len(parts) == 4:
                        try:
                            pressure    = float(parts[1])
                            temperature = float(parts[2])
                            depth       = float(parts[3])

                            p_msg = Float32(); p_msg.data = pressure
                            t_msg = Float32(); t_msg.data = temperature
                            d_msg = Float32(); d_msg.data = depth

                            self.pub_pressure.publish(p_msg)
                            self.pub_temperature.publish(t_msg)
                            self.pub_depth.publish(d_msg)

                            self.get_logger().info(
                                f'Pressure: {pressure:.2f} mbar | '
                                f'Temp: {temperature:.2f} °C | '
                                f'Depth: {depth:.3f} m'
                            )
                        except ValueError:
                            self.get_logger().warn(f'Bad sensor line: {line}')
                    else:
                        self.get_logger().warn(f'Unexpected sensor format: {line}')
                else:
                    # Any other Arduino debug print
                    self.get_logger().info(f'[Arduino] {line}')

            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'Reader thread error: {e}')
                break

    # ── Cleanup ───────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('Shutting down — neutral thrusters + lights off...')
        self.stop_flag.set()
        try:
            self.ser.write(b"1500,1500,1500,1500,1500,1500,1500,1500\n")
            self.ser.write(b"L,1100,1100\n")
            self.ser.flush()
            time.sleep(0.2)
            self.ser.close()
            self.get_logger().info('Serial closed.')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AUVBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
