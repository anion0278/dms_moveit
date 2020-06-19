#!/usr/bin/env python

import roslaunch
import ros_process
import rospy
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
import time

def __mainloop():
        print("Disconnecting all BLE devices...")
        ble.clear_cached_data()
        # Get the first available BLE network adapter and make sure it"s powered on.
        adapter = ble.get_default_adapter()
        adapter.power_on()
        UART.disconnect_devices()
        adapter.start_scan()
        adapter.stop_scan()
        print("All BLE devices desconnected")

ble = Adafruit_BluefruitLE.get_provider()
ble.initialize()
ble.run_mainloop_with(__mainloop)