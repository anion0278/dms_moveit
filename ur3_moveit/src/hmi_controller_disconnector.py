#!/usr/bin/env python

import roslaunch
import rospy
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
import time
import os

def restart_adapter():
        print("Restarting adapter...")
        os.system("rfkill block bluetooth")
        time.sleep(0.5)
        os.system("rfkill unblock bluetooth")
        time.sleep(0.5)
        print("Adapter restarted")

def disconnect_all_devices():
        print("Disconnecting all BLE devices...")
        ble.clear_cached_data()
        adapter = ble.get_default_adapter()
        adapter.power_on()
        UART.disconnect_devices()
        adapter.start_scan()
        adapter.stop_scan()
        print("All BLE devices disconnected")

def __mainloop():
        restart_adapter()
        disconnect_all_devices()



if __name__ == "__main__":
        ble = Adafruit_BluefruitLE.get_provider()
        ble.initialize()
        ble.run_mainloop_with(__mainloop)