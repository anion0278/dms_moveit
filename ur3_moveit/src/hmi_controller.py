#!/usr/bin/env python

import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
from datetime import datetime
import rospy
import numpy as np
import sys
import signal
import atexit
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty
import os
import time
import re
import config

debug = False  

obj_clearance_param = "/move_group/collision/min_clearance"

class TimedCommand():
    def __init__(self, intensity, time):
        self.time = time
        self.intensity = intensity

def __recieved_callback_1(data):
    print(data)

class HmiController():
    def __init__(self,
                 device_name,
                 clearance_hand_suffix,
                 rosparam_name="hmi_value"):
        self.node_name = device_name.replace("-", "_")
        rospy.init_node(self.node_name)
        self.device = None
        self.uart = None
        self.ble = None
        self.device_name = device_name
        self.this_hand_clearance_param = obj_clearance_param + clearance_hand_suffix
        self.second_hand_clearance_param = self.__get_second_hand_name(self.this_hand_clearance_param)
        self.hmi_status_param = rosparam_name
        rospy.set_param(rosparam_name, 0)
        self.timeouts = 1
        self.clearance_min = 0.15
        self.min_vib = config.dist_intensity_min
        self.max_vib = config.dist_intensity_max

    def send_speed_command(self, speed):
        rospy.set_param("debug_"+self.node_name, speed)
        speed_text = str(int(speed)).zfill(3)
        self.send_text("C{}\r\n".format(speed_text))
        rospy.sleep(rospy.Duration(secs=0, nsecs=500))

    def send_text(self, text):
        try:
            self.uart.write(text)
            if debug:
                print(self.__get_time() + " Sent:" + text)
        except Exception as ex:
            print("SENDING {" + text + "} EXCEPTION: " + ex)

    def start(self):
        self.ble = Adafruit_BluefruitLE.get_provider()
        self.ble.initialize()
        self.ble.run_mainloop_with(self.__mainloop)

    @staticmethod
    def __recieved_callback(data):
        if len(data) > 15:
            regex_pattern = "Q\[(-?\d+.\d+); (-?\d+.\d+); (-?\d+.\d+); (-?\d+.\d+)]-C\[(\d); (\d); (\d); (\d)]"
            m = re.search(regex_pattern, data, re.IGNORECASE)
            if m: 
                orientation = Quaternion(m.group(2), m.group(3), m.group(4), m.group(1))
                calibration = [m.group(5), m.group(6), m.group(7), m.group(8)]
                pass
            print(data)

    def __mainloop(self):
        rospy.set_param(self.hmi_status_param, 0)
        rospy.set_param(obj_clearance_param, self.clearance_min)
        rospy.set_param(self.this_hand_clearance_param, self.clearance_min)
        rospy.set_param(self.second_hand_clearance_param, self.clearance_min)
        atexit.register(self.__on_exit)

        self.ble.clear_cached_data()
        # Get the first available BLE  adapter 
        adapter = self.ble.get_default_adapter()
        print("Using adapter: {0}".format(adapter.name))
        adapter.power_on()

        try:
            print("Searching for BLE UART device...")
            adapter.start_scan(timeout_sec=self.timeouts)
            time.sleep(1)
            devs = UART.find_devices()
            device = next((d for d in devs if d.name == self.device_name))
            print("Connecting to device...")
            device.connect(timeout_sec=self.timeouts)
        except Exception as ex:
            print("Failed to connect to UART device! %s" % ex)
            UART.disconnect_devices()
        finally:
            adapter.stop_scan()

        print("Discovering services...")
        UART.discover(device, timeout_sec=self.timeouts)
        self.uart = UART(device, HmiController.__recieved_callback)

        self.__notify_ready()

        time_step = 0.05

        while True:
            command = self.calc_intensity()
            if isinstance(command, TimedCommand):
                while command.time > 0:
                    self.send_speed_command(command.intensity)
                    rospy.sleep(time_step)
                    command.time = command.time - time_step
                    new_command = self.calc_intensity()
                    if (isinstance(new_command, TimedCommand) and new_command.intensity > command.intensity)  or new_command > command.intensity:
                        if isinstance(new_command, TimedCommand):
                            self.send_speed_command(new_command.intensity)
                        else:
                            self.send_speed_command(new_command)
                        break

            else:
                self.send_speed_command(command)

    def calc_intensity(self):
        # TODO VIBRATION should be proportional to the distance to the future trajectory AND !
        # AND proportional to the distance to the robot - if the user is in the path of trajectory,
        # then he still has time to react, no need to vibrate intesivelly. Requires changes in MoveIt!
        hmi_status_command =  rospy.get_param(self.hmi_status_param)
        if hmi_status_command != 0:
            if hmi_status_command == config.status_invalid:
                return config.invalid_goal_intensity

            if hmi_status_command == config.status_replan:
                return TimedCommand(config.replan_intensity, 0.3)    
        else:
            this_hand_clearance_level = rospy.get_param(self.this_hand_clearance_param)
            second_hand_clearance_level = rospy.get_param(self.second_hand_clearance_param)
            objs_clearance_level = rospy.get_param(obj_clearance_param)

            # choose the smallest value if the smallest value is not the same as for second hand
            if objs_clearance_level < this_hand_clearance_level and second_hand_clearance_level != objs_clearance_level:  
                this_hand_clearance_level = objs_clearance_level

            if this_hand_clearance_level < self.clearance_min:
                vibration_level = (self.max_vib - self.min_vib) * (self.clearance_min -
                    this_hand_clearance_level) / self.clearance_min + self.min_vib
                return vibration_level
            else:
                return 0

    def __notify_ready(self):
        # service notifying that HMI is ready
        self.ready_service = rospy.Service(self.node_name + "_service", Empty,None)
        self.send_speed_command(0)
        print("Device %s is ready." % self.device_name)

    def read_with_timeout(self, timeout_sec=1):
        received = self.uart.read(timeout_sec=timeout_sec)
        if received is not None:
            print("Received: {0}".format(received))
        else:
            print("Received no data!")

    def __get_second_hand_name(self, first_hand_name):
        print(first_hand_name)
        if "left" in first_hand_name:
            return first_hand_name.replace("left", "right")
        if "right" in first_hand_name:
            return first_hand_name.replace("right", "left")
        raise AttributeError("Check hand name!")

    #def __on_data_recieved(self, data):
    #    print(self.__get_time() + " Recieved:" + data)

    def __on_exit(self):
        print("Disconnecting... *It is possible to set lower timeout*")
        try:
            from Adafruit_BluefruitLE.services import UART
            # it is not possible to connect again after disconnection!!! Only restart
            UART.disconnect_devices()
        except:
            pass  # nevermind it

    def __get_time(self):
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]


if __name__ == "__main__":
    sys.argv.append("hmi-glove-left")
    sys.argv.append("_left")
    # for arg, i in zip(sys.argv, range(len(sys.argv))):
    #     print("Arg [%s]: %s" % (i, arg))
    hmi = HmiController(sys.argv[1], sys.argv[2])
    hmi.start()