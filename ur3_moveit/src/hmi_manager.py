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

os.setpgrp()

adapter = None
global device
device = None

def on_exit():
    print("Disconnecting... *It is possible to set lower timeout*")
    from Adafruit_BluefruitLE.services import UART
    #try:
    UART.disconnect_devices()
    #except:
    #    pass # nevermind


# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()

def get_time():
    return datetime.now().strftime("%d-%b-%Y-%H-%M-%S.%f")[:-3]


def ble_data_recieved(data):
    print(get_time() + " Callback:" + data)


# Main function implements the program logic so it can run in a background
# thread.  Most platforms require the main thread to handle GUI events and other
# asyncronous events like BLE actions.  All of the threading logic is taken care
# of automatically though and you just need to provide a main function that uses
# the BLE provider.
def main():
    rospy.set_param("hmi_value", 0)

    # Clear any cached data because both bluez and CoreBluetooth have issues with
    # caching data and it going stale.
    ble.clear_cached_data()

    # Get the first available BLE network adapter and make sure it"s powered on.
    adapter = ble.get_default_adapter()
    adapter.power_on()
    print("Using adapter: {0}".format(adapter.name))

    # Disconnect any currently connected UART devices.  Good for cleaning up and
    # starting from a fresh state.
    print("Disconnecting any connected UART devices...")
    UART.disconnect_devices()

    # Scan for UART devices.
    print("Searching for UART device...")
    try:
        adapter.start_scan()
        # Search for the first UART device found (will time out after 60 seconds
        # but you can specify an optional timeout_sec parameter to change it).
        device = UART.find_device()
        if device is None:
            raise RuntimeError("Failed to find UART device!")
    finally:
        # Make sure scanning is stopped before exiting.
        adapter.stop_scan()

    print("Connecting to device...")
    device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                    # to change the timeout.
    atexit.register(on_exit)
    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        # Wait for service discovery to complete for the UART service.  Will
        # time out after 60 seconds (specify timeout_sec parameter to override).
        print("Discovering services...")
        UART.discover(device)

        # Once service discovery is complete create an instance of the service
        # and start interacting with it.
        uart = UART(device, receive_callback=ble_data_recieved)

        while True:
            hmi_command = rospy.get_param("hmi_value")
            if hmi_command != 0:
                uart.write("C"+str(hmi_command) +"\r\n")
                print(get_time() +" Sent " + str(hmi_command))
                rospy.sleep(rospy.Duration(secs=0, nsecs=500)) # 50 ms
            else:
                clearance_level = rospy.get_param("/move_group/collision/min_clearance")
                clearance_min = 0.15
                min_vib = 50
                max_vib = 100
                if clearance_level < clearance_min:
                    vibration_level = (max_vib - min_vib) * (clearance_min - clearance_level) / clearance_min + min_vib
                    uart.write("C{:0f}\r\n".format(vibration_level)) 
                    print("send {:0f}".format(vibration_level))
                else: 
                    uart.write("C000\r\n") 

        # Now wait up to one minute to receive data from the device.
        print("Waiting up to 60 seconds to raeceive data from the device...")
        received = uart.read(timeout_sec=60)
        if received is not None:
            # Received data, print it out.
            print("Received: {0}".format(received))
        else:
            # Timeout waiting for data, None is returned.
            print("Received no data!")
    finally:
        # Make sure device is disconnected on exit.
        device.disconnect()

# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)