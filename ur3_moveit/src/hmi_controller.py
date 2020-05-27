#!/usr/bin/env python

import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
from datetime import datetime
import rospy
import numpy as np
import sys
import signal
import atexit
import os


class HmiController():
    def __init__(self, rosparam_name = "hmi_value"):
        self.device = None
        self.uart = None
        self.ble = None
        self.param_name = rosparam_name
        rospy.set_param(rosparam_name, 0)
        self.timeouts = 10
        self.clearance_min = 0.15
        self.min_vib = 50
        self.max_vib = 100

    def get_speed(self):
        return rospy.get_param(self.param_name)

    def send_speed_command(self, speed):
        speed_text = str(int(speed)).zfill(3)
        self.send_text("C{}\r\n".format(speed_text))
        rospy.sleep(rospy.Duration(secs=0, nsecs=500))

    def send_text(self, text):
        print(self.__get_time() + " Sent:" + text)
        self.uart.write(text)

    def start(self):
        self.ble = Adafruit_BluefruitLE.get_provider()
        self.ble.initialize()
        self.ble.run_mainloop_with(self.__mainloop)

    def __mainloop(self):
        rospy.set_param(self.param_name, 0)
        rospy.set_param("/move_group/collision/min_clearance", self.clearance_min)
        atexit.register(self.__on_exit)

        self.ble.clear_cached_data()

        # Get the first available BLE network adapter and make sure it"s powered on.
        adapter = self.ble.get_default_adapter()
        print("Using adapter: {0}".format(adapter.name))
        adapter.power_on()

        print("Disconnecting any connected UART devices...")
        UART.disconnect_devices()

        # Scan for UART devices.
        print("Searching for BLE UART device...")
        try:
            adapter.start_scan()
            device = UART.find_device(timeout_sec=self.timeouts)
            if device is None:
                raise RuntimeError("Failed to find UART device!")
        finally:
            adapter.stop_scan()

        print("Connecting to device...")
        device.connect(timeout_sec=self.timeouts) 
        
        print("Discovering services...")
        UART.discover(device, timeout_sec=self.timeouts)

        self.uart = UART(device, receive_callback=self.__on_data_recieved)

        self.send_speed_command(0)

        # TODO VIBRATION should be proportional to the distance to the future trajectory AND !
        # AND proportional to the distance to the robot - if the user is in the path of trajectory, 
        # then he still has time to react, no need to vibrate intesivelly
        # TODO refactoring
        while True:
            hmi_command = self.get_speed()
            if hmi_command != 0:
                self.send_speed_command(hmi_command)
            else:
                clearance_level = rospy.get_param("/move_group/collision/min_clearance")

                if clearance_level < self.clearance_min:
                    vibration_level = (self.max_vib - self.min_vib) * (self.clearance_min - clearance_level) / self.clearance_min + self.min_vib
                    self.send_speed_command(vibration_level)
                else: 
                    self.send_speed_command(0)

    def read_with_timeout(self, timeout_sec = 1):
        received = self.uart.read(timeout_sec=timeout_sec)
        if received is not None:
            print("Received: {0}".format(received))
        else:
            print("Received no data!")

    def __on_data_recieved(self, data):
        print(self.__get_time() + " Recieved:" + data)

    def __on_exit(self):
        print("Disconnecting... *It is possible to set lower timeout*")
        try:
            from Adafruit_BluefruitLE.services import UART
            UART.disconnect_devices()
        except:
            pass # nevermind it

    def __get_time(self):
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

if __name__ == "__main__":
    hmi = HmiController()
    hmi.start()