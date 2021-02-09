#!/usr/bin/env python

import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
import sys
#import signal # USELESS works only in the main thread
import atexit
import os
import time
import threading
import util_timeout

import util_common as util

class BleManager():
    def __init__(self, device_name, debug, mainthread_callback, on_data_callback):
        self.device_name = device_name
        self.__uart = None
        self.__ble = None
        self.__timeouts = 1
        self.__semaphore = threading.Semaphore()
        self.debug = debug
        self.mainthread_callback = mainthread_callback
        self.on_data_callback = on_data_callback
 
    def start(self):
        self.__ble = Adafruit_BluefruitLE.get_provider()
        self.__ble.initialize()
        self.__ble.run_mainloop_with(self.__mainloop)

    def __mainloop(self):
        atexit.register(self.__on_exit)
        self.__connect_hmi()
        self.mainthread_callback()

    def send_text(self, text, newline = True):
        if newline:
            text += "\r\n" 
        self.__semaphore.acquire()
        # solved by killing HMI Controllers' processes in watchdog
        self.__uart.write(text) # has at least 30s timeout, blocks the thread, causes problems when HMI is improperly disconnected
        self.__semaphore.release()
        if self.debug:
            print("Device: %s Time: %s Sent: %s" % (self.device_name, util.get_time_str(), text))

    def __connect_hmi(self):
        self.__ble.clear_cached_data()
        adapter = self.__ble.get_default_adapter()
        print("Using adapter: {0}".format(adapter.name))
        adapter.power_on()

        success = False
        while not success: # sometimes BLE device does not connect immediatelly
            try:
                print("Searching for BLE UART device (%s)..." % self.device_name)
                adapter.start_scan(timeout_sec=self.__timeouts)
                time.sleep(1)
                devs = UART.find_devices()
                device = next((d for d in devs if d.name == self.device_name))

                print("Connecting to device (%s)..." % self.device_name)
                device.connect(timeout_sec=self.__timeouts)

                print("Discovering services (UART)...")
                UART.discover(device, timeout_sec=self.__timeouts)
                self.__uart = UART(device, self.on_data_callback)
                
                success = True
            except Exception as ex:
                print("Failed to connect to %s, reason: %s" % (self.device_name, ex))
            finally:
                adapter.stop_scan()

    def __on_exit(self):
        print("BLE Exit-handler...")
        try:
            from Adafruit_BluefruitLE.services import UART
            UART.disconnect_devices()
        except:
            pass  # if was not connected
    