#!/usr/bin/env python3

"""ROS2 node for Cerulean Omniscan 450 FS forward-scan sonar."""

import time

import rclpy
from rclpy.node import Node

from frontscan_ros2.msg import FrontScanSonar, FrontScanSonarRaw
from frontscan_ros2_nodes.omniscan450_driver import Omniscan450Driver
from frontscan_ros2_nodes.sonar_params import (
    declare_sonar_params,
    get_sonar_params,
    make_param_callback,
    timer_period_sec,
)

SENSOR_NAME = 'frontscan'
SONAR_TOPIC_NAME = 'range'
SONAR_RAW_TOPIC_NAME = 'range_raw'

SENSOR_NUMBER = '450'
IP_ADDRESS = '192.168.194.90'
SONAR_PORT = 51200


def _populate_msgs(sonar_msg, sonar_raw_msg, stamp, data, scaled):
    sonar_raw_msg.header.stamp = stamp
    sonar_raw_msg.header.frame_id = 'frontscan'
    sonar_raw_msg.ping_number = data.ping_number
    sonar_raw_msg.start_mm = data.start_mm
    sonar_raw_msg.length_mm = data.length_mm
    sonar_raw_msg.timestamp_ms = data.timestamp_ms
    sonar_raw_msg.ping_hz = data.ping_hz
    sonar_raw_msg.gain_index = data.gain_index
    sonar_raw_msg.num_results = data.num_results
    sonar_raw_msg.sos_dmps = data.sos_dmps
    sonar_raw_msg.channel_number = data.channel_number
    sonar_raw_msg.pulse_duration_sec = data.pulse_duration_sec
    sonar_raw_msg.analog_gain = data.analog_gain
    sonar_raw_msg.max_pwr_db = data.max_pwr_db
    sonar_raw_msg.min_pwr_db = data.min_pwr_db
    sonar_raw_msg.transducer_heading_deg = data.transducer_heading_deg
    sonar_raw_msg.vehicle_heading_deg = data.vehicle_heading_deg
    sonar_raw_msg.pwr_data = list(data.pwr_results)

    sonar_msg.header.stamp = stamp
    sonar_msg.header.frame_id = 'frontscan'
    sonar_msg.ping_number = data.ping_number
    sonar_msg.start_mm = data.start_mm
    sonar_msg.length_mm = data.length_mm
    sonar_msg.timestamp_ms = data.timestamp_ms
    sonar_msg.ping_hz = data.ping_hz
    sonar_msg.gain_index = data.gain_index
    sonar_msg.num_results = data.num_results
    sonar_msg.sos_dmps = data.sos_dmps
    sonar_msg.channel_number = data.channel_number
    sonar_msg.pulse_duration_sec = data.pulse_duration_sec
    sonar_msg.analog_gain = data.analog_gain
    sonar_msg.max_pwr_db = data.max_pwr_db
    sonar_msg.min_pwr_db = data.min_pwr_db
    sonar_msg.transducer_heading_deg = data.transducer_heading_deg
    sonar_msg.vehicle_heading_deg = data.vehicle_heading_deg
    sonar_msg.scaled_data = list(scaled)


class FrontscanNode(Node):
    def __init__(self):
        super().__init__('frontscan')

        sensor_number = str(self.declare_parameter('sensor_number', int(SENSOR_NUMBER)).value)
        ip_address = str(self.declare_parameter('ip_address', IP_ADDRESS).value)
        sonar_port = self.declare_parameter('sonar_port', SONAR_PORT).value

        declare_sonar_params(self)
        sonar_params = get_sonar_params(self)

        self.sensor_name = SENSOR_NAME + sensor_number
        topic_base = '/' + self.sensor_name
        self.range_raw_pub = self.create_publisher(
            FrontScanSonarRaw, topic_base + '/' + SONAR_RAW_TOPIC_NAME, 10)
        self.range_pub = self.create_publisher(
            FrontScanSonar, topic_base + '/' + SONAR_TOPIC_NAME, 10)

        logger = self.get_logger()
        logger.info(f'Front-scan driver targeting {ip_address}:{sonar_port}')

        self.driver = Omniscan450Driver(
            ip_address=ip_address, port=sonar_port, logger=logger, **sonar_params)

        self.add_on_set_parameters_callback(
            make_param_callback([self.driver], self))

        self.first_exception_logged = False
        self._timer = self.create_timer(
            timer_period_sec(sonar_params['msec_per_ping']), self._poll_callback)

        time.sleep(1)

    def _poll_callback(self):
        current_time = self.get_clock().now().to_msg()

        try:
            data, scaled_results = self.driver.get_data()
            if data is None:
                return

            sonar_msg = FrontScanSonar()
            sonar_raw_msg = FrontScanSonarRaw()
            _populate_msgs(sonar_msg, sonar_raw_msg, current_time, data, scaled_results)

            self.range_raw_pub.publish(sonar_raw_msg)
            self.range_pub.publish(sonar_msg)
            self.first_exception_logged = False

        except Exception:
            if not self.first_exception_logged:
                self.get_logger().error('Exception when reading Omniscan FS sonar data.')
                self.first_exception_logged = True

    def destroy_node(self):
        self.driver.close_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontscanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
