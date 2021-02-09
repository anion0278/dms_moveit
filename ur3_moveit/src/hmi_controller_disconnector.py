#!/usr/bin/env python

import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
import time
import os
import sys
import setproctitle

import util_ros_process 
import config

class HmiDisconnector():
        def __init__(self, use_exit):
                self.ble = None
                self.use_exit = use_exit

        def run(self):
                self.ble = Adafruit_BluefruitLE.get_provider()
                self.ble.initialize()
                self.ble.run_mainloop_with(self.__mainloop)
                if self.use_exit:
                        sys.exit() 

        def restart_adapter(self):
                print("Restarting adapter...")
                os.system("rfkill block bluetooth")
                time.sleep(0.5)
                os.system("rfkill unblock bluetooth")
                time.sleep(0.5)
                print("Adapter restarted")

        def disconnect_all_devices(self):
                print("Disconnecting all BLE devices...")
                self.ble.clear_cached_data()
                adapter = self.ble.get_default_adapter()
                adapter.power_on()
                UART.disconnect_devices()
                adapter.start_scan()
                adapter.stop_scan()
                print("All BLE devices disconnected")

        def __mainloop(self):
                self.restart_adapter()
                self.disconnect_all_devices()


def run_full_disconnection(use_exit = False):
        util_ros_process.kill_processes_by_name(config.hmi_disconnector_process)
        util_ros_process.kill_processes_by_name(config.hmi_watchdog_process)
        util_ros_process.kill_processes_by_contained_name(config.hmi_controller_process)
        setproctitle.setproctitle(config.hmi_disconnector_process)
        d = HmiDisconnector(use_exit)
        d.run()

if __name__ == "__main__":
        run_full_disconnection()