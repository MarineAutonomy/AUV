#!/usr/bin/env python3

"""
  Author: Monika Roznere
  Affiliation: Binghamton University
  Date: 07/19/2025

  Description:
  Class for interacting with Omniscan 450 Side Scan Sonar.
"""

import time

from brping import definitions
from brping import Omniscan450

SONAR_IP_ADDRESS = "192.168.2.92"
SONAR_PORT = 51200
SPEED_OF_SOUND_MM = 1482000
START_MM = 0
LENGTH_MM = 5000
MSEC_PER_PING = 0
PULSE_LEN_PERCENT = 0.002
FILTER_DURATION_PERCENT = 0.0015
GAIN_INDEX = -1
NUM_RESULTS = 600


class Omniscan450Driver:
    def __init__(self, ip_address=SONAR_IP_ADDRESS, port=SONAR_PORT,
                 speed_of_sound_mm=SPEED_OF_SOUND_MM, start_mm=START_MM, length_mm=LENGTH_MM,
                 msec_per_ping=MSEC_PER_PING, pulse_len_percent=PULSE_LEN_PERCENT,
                 filter_duration_percent=FILTER_DURATION_PERCENT,
                 gain_index=GAIN_INDEX, num_results=NUM_RESULTS, logger=None):

        self.ip_address = ip_address
        self.logger = logger

        self.speed_of_sound_mm = speed_of_sound_mm
        self.start_mm = start_mm
        self.length_mm = length_mm
        self.msec_per_ping = msec_per_ping
        self.pulse_len_percent = pulse_len_percent
        self.filter_duration_percent = filter_duration_percent
        self.gain_index = gain_index
        self.num_results = num_results

        self.omniscan450 = Omniscan450()

        self.omniscan450.connect_tcp(self.ip_address, port)
        if self.omniscan450.initialize() is False:
            self._log("Omniscan 450 SS Port %s is closed" % self.ip_address)

        data = self.omniscan450.readDeviceInformation()
        self._log("Device type: %s" % data.device_type)

        self._set_sidescan_params(change_speed_of_sound=True, change_os_ping_params=True)

    def _log(self, message):
        if self.logger is not None:
            self.logger.info(message)
        else:
            print(message)

    def get_data(self):
        """Get data from the sonar and return raw data and scaled result."""
        data = self.omniscan450.wait_message([definitions.OMNISCAN450_OS_MONO_PROFILE])
        if not data:
            self._log("Omniscan 450 SS Port %s failed to get message" % self.ip_address)
            return None, None

        scaled_results = Omniscan450.scale_power(data)
        try:
            sum(scaled_results) / len(scaled_results)
        except ZeroDivisionError:
            self._log("Omniscan 450 SS Port %s : length of scaled result is 0" % self.ip_address)

        return data, scaled_results

    def set_parameters(self, config):
        """Set parameters from a dict of sonar tuning values."""
        change_speed_of_sound = False
        if self.speed_of_sound_mm != config['speed_of_sound_mm']:
            self.speed_of_sound_mm = config['speed_of_sound_mm']
            change_speed_of_sound = True

        change_os_ping_params = False
        if self.start_mm != config['start_mm']:
            self.start_mm = config['start_mm']
            change_os_ping_params = True

        if self.length_mm != config['length_mm']:
            self.length_mm = config['length_mm']
            change_os_ping_params = True

        if self.msec_per_ping != config['msec_per_ping']:
            self.msec_per_ping = config['msec_per_ping']
            change_os_ping_params = True

        if self.pulse_len_percent != config['pulse_len_percent']:
            self.pulse_len_percent = config['pulse_len_percent']
            change_os_ping_params = True

        if self.filter_duration_percent != config['filter_duration_percent']:
            self.filter_duration_percent = config['filter_duration_percent']
            change_os_ping_params = True

        if self.gain_index != config['gain_index']:
            self.gain_index = config['gain_index']
            change_os_ping_params = True

        if self.num_results != config['num_results']:
            self.num_results = config['num_results']
            change_os_ping_params = True

        self._set_sidescan_params(change_speed_of_sound, change_os_ping_params)
        return config

    def _set_sidescan_params(self, change_speed_of_sound=False, change_os_ping_params=False):
        if change_speed_of_sound:
            self.omniscan450.control_set_speed_of_sound(
                speed_of_sound=self.speed_of_sound_mm
            )

        if change_os_ping_params:
            self.omniscan450.control_os_ping_params(
                start_mm=self.start_mm,
                length_mm=self.length_mm,
                msec_per_ping=self.msec_per_ping,
                pulse_len_percent=self.pulse_len_percent,
                filter_duration_percent=self.filter_duration_percent,
                gain_index=self.gain_index,
                num_results=self.num_results,
                enable=1,
                reserved_1=0,
                reserved_2=0,
                reserved_3=0,
            )

        time.sleep(0.01)

    def close_connection(self):
        self.omniscan450.control_os_ping_params(
            start_mm=self.start_mm,
            length_mm=self.length_mm,
            msec_per_ping=self.msec_per_ping,
            pulse_len_percent=self.pulse_len_percent,
            filter_duration_percent=self.filter_duration_percent,
            gain_index=self.gain_index,
            num_results=self.num_results,
            enable=0,
            reserved_1=0,
            reserved_2=0,
            reserved_3=0,
        )

        if self.omniscan450.iodev:
            try:
                self.omniscan450.iodev.close()
            except Exception as e:
                self._log("Failed to close socket: %s" % e)
