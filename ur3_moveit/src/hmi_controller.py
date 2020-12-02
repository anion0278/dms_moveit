#!/usr/bin/env python

import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
from datetime import datetime
import rospy
import numpy as np
import sys
import signal
import atexit
from geometry_msgs.msg import *
import tf2_geometry_msgs as tf2
from std_srvs.srv import Empty
import os
import time
import re
from functools import partial
from tf.transformations import *
from std_srvs.srv import Trigger
import ros_numpy
import tf, tf2_ros

import config
import hmi_visualisation as vis


debug = False  
obj_clearance_param = "/move_group/collision/min_clearance"
regex_msg_pattern = "Q(-?\d)(\d{2})(-?\d)(\d{2})(-?\d)(\d{2})(-?\d)(\d{2})(\d)(\d)(\d)(\d)"

class SensorCalibration():
    def __init__(self, sys, gyro, acc, mag):
        self.sys = int(sys)
        self.gyro = int(gyro)
        self.acc = int(acc)
        self.mag = int(mag)
    
    @property
    def is_ready(self):
        #Therefore we recommend that as long as Magnetometer is 3/3, and Gyroscope is 3/3, the data can be trusted
        if self.sys == 3 and self.gyro == 3 and self.acc == 3 and self.mag == 3:
            pass

    def __str__(self):
        return "Sys: {0}; Gyro: {1}; Acc: {2}; Mag: {3}".format(self.sys, self.gyro, self.acc, self.mag)

class TimedCommand():
    def __init__(self, intensity, time):
        self.time = time
        self.intensity = intensity

class HmiController():
    def __init__(self,
                 device_name,
                 clearance_hand_suffix, 
                 is_calib_enabled,
                 rosparam_name="hmi_value"): # TODO PUT INTO CONFIG EVERYWHERE!!!
        self.node_name = device_name.replace("-", "_")
        rospy.init_node(self.node_name)
        self.orient_pub = rospy.Publisher(self.node_name + "_orientation", PoseStamped, queue_size=1)
        self.device = None
        self.__uart = None
        self.ble = None
        self.device_name = device_name
        self.this_hand_clearance_param = obj_clearance_param + clearance_hand_suffix
        self.second_hand_clearance_param = self.__get_second_hand_name(self.this_hand_clearance_param)
        self.hmi_status_param = rosparam_name
        rospy.set_param(rosparam_name, 0) # TODO abstraction 
        self.timeouts = 1
        self.clearance_min = 0.15 
        self.__min_vib = config.dist_intensity_min
        self.__max_vib = config.dist_intensity_max
        self.current_orientation = None
        self.__calib_quality_counter = 0
        self.__imu_offsets = None
        self.is_calib_enabled = True
        self.zero_frame_clib = None
        self.tf_pub = tf2_ros.TransformBroadcaster() # tf2_ros pubs are more effective
        self.__real_frame = self.device_name+"_real"
        self.__calibr_frame = self.device_name+"_offset"
        self.calibr_service = None # service for calibrating Frame
        if "left" in self.device_name: #TODO put this logic into config
            color = config.color_left
        if "right" in self.device_name:
            color = config.color_right
        color.append(0.7) # alpha
        self.__visualizer = vis.RVizVisualiser(color, self.node_name + "_markers", self.__calibr_frame, 1.0)

    def __get_speed_components(self, speed = 100):
        speed_comps=[0,0,0,0,0,0]
        if self.current_orientation is not None: 

            vs = Vector3Stamped(vector = Vector3(x = 1, y = 0, z = 0))
            # needed an inverse matrix!  
            
            qa = quaternion_inverse(ros_numpy.numpify(self.current_orientation))
            trs = TransformStamped(transform = Transform(rotation = Quaternion(*qa)))
            v = tf2.do_transform_vector3(vs, trs).vector
            va = speed * ros_numpy.numpify(v)

            self.__visualizer.publish_if_required(v)

            # components x, y, z, -x, -y, -z
            for i in range(len(speed_comps) / 2):
                if va[i] > 0: # positive
                    speed_comps[i] = va[i]
                else: # negative number
                    speed_comps[i + 3] = abs(va[i])
            #print(speed_comps)
        return speed_comps

    def send_speed_command(self, speed):
        rospy.set_param("debug_"+self.node_name, speed)
        speeds = self.__format_speed_msg(self.__get_speed_components())
        msg ="X{0}Y{1}Z{2}-X{3}-Y{4}-Z{5}\r\n".format(*speeds) 
        self.send_text(msg)
        rospy.sleep(rospy.Duration(secs=0, nsecs=5000))

    def __format_speed_msg(self, speed_comps):
        for i in range(len(speed_comps)):
            val = speed_comps[i]
            if speed_comps[i] < config.vibr_min and speed_comps[i] > config.vibr_min / 2:
                val = config.vibr_min
            speed_comps[i] = str(int(val)).zfill(3)
        return speed_comps

    def send_handshake(self):
        self.send_text("handshake:%s\r\n" % int(self.is_calib_enabled))

    def send_text(self, text):
        try:
            self.__uart.write(text)
            if debug:
                print(self.__get_time() + " Sent:" + text)
        except Exception as ex:
            print("SENDING {" + text + "} EXCEPTION: " + ex.message)

    def start(self):
        self.ble = Adafruit_BluefruitLE.get_provider()
        self.ble.initialize()
        self.ble.run_mainloop_with(self.__mainloop)

    def __recieved_callback(self, data):
        #print(data)
        match = re.search(regex_msg_pattern, data, re.IGNORECASE)
        if match: 
            da = np.array(match.groups())

            print(self.node_name +"->"+data)
            p = PoseStamped()
            #p.header.seq = 1 # just ID
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "world"

            q_real = ros_numpy.numpify(self.__get_compressed_quarternion(da))
            inv = quaternion_inverse(ros_numpy.numpify(self.__get_compressed_quarternion(da)))
            calib = [0.02982404,  0.00994135, -0.99413461, -0.06958942]
            current = quaternion_multiply(q_real, calib)
            #current = ros_numpy.numpify(self.__get_compressed_quarternion(da))
            #ROS quaternion -> x,y,z,w
            p.pose.orientation = Quaternion(*current)
            self.current_orientation = p.pose.orientation

            tfs = TransformStamped(transform = Transform(rotation = Quaternion(*q_real)))
            tfs.header.stamp = rospy.Time.now()
            tfs.header.frame_id = "world"
            tfs.child_frame_id = self.device_name+"_real"
            self.tf_pub.sendTransform(tfs)

            # tfs.transform.rotation = p.pose.orientation
            # tfs.child_frame_id = self.device_name+"_offset"
            # self.tf_pub.sendTransform(tfs)

            # both options work correctly, however this one is correct structure
            # publish only if Debug? need it for Markers
            tfs.transform.rotation = Quaternion(*calib)
            tfs.header.frame_id = self.device_name+"_real"
            tfs.child_frame_id = self.device_name+"_offset"
            self.tf_pub.sendTransform(tfs)

            if (self.orient_pub.get_num_connections() > 0):
                self.orient_pub.publish(p)

            calibration = SensorCalibration(da[8], da[9], da[10], da[11])
            print(str(calibration))
            pass 

    def __get_compressed_quarternion(self, data):
        floats = [0,0,0,0]
        for i in range(len(floats)):
            floats[i] = float(data[i * 2]+'.'+data[i * 2 + 1])
        q = Quaternion(*floats)
        return q

    def __mainloop(self):
        # TODO wrap external ropspy calls into methods
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

        success = False
        while not success:
            try:
                print("Searching for BLE UART device...")
                adapter.start_scan(timeout_sec=self.timeouts)
                time.sleep(1)
                devs = UART.find_devices()
                device = next((d for d in devs if d.name == self.device_name))
                print("Connecting to device...")
                device.connect(timeout_sec=self.timeouts)
                success = True
            except Exception as ex:
                print("Failed to connect to UART device! %s" % ex)
                UART.disconnect_devices()
            finally:
                adapter.stop_scan()
            
        print("Discovering services...")
        UART.discover(device, timeout_sec=self.timeouts)
        self.__uart = UART(device, self.__recieved_callback)

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
                vibration_level = (self.__max_vib - self.__min_vib) * (self.clearance_min -
                    this_hand_clearance_level) / self.clearance_min + self.__min_vib
                return vibration_level
            else:
                return 0

    def __notify_ready(self):
        self.send_handshake()
        # service notifying that HMI is ready
        self.calibr_service = rospy.Service(self.node_name + "_service", Empty, None)
        self.send_speed_command(0)
        print("Device %s is ready." % self.device_name)

    def read_with_timeout(self, timeout_sec=1):
        received = self.__uart.read(timeout_sec=timeout_sec)
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

    def __on_exit(self): # TODO investigate
        print("Disconnecting... *It is possible to set lower timeout in uart.py*")
        try:
            from Adafruit_BluefruitLE.services import UART
            UART.disconnect_devices()
        except:
            pass  # if did not connect

    def __get_time(self):
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]


if __name__ == "__main__":

    #### causes problems TODO solve
    if not "node" in sys.argv:
        print("DEBUGGER MODE !!! Will cause error in roslaunch!")
        os.system("rfkill block bluetooth")
        time.sleep(0.5)
        os.system("rfkill unblock bluetooth")
        sys.argv.append("hmi-glove-left")
        sys.argv.append("_left")

    for arg, i in zip(sys.argv, range(len(sys.argv))):
        print("Arg [%s]: %s" % (i, arg))
    hmi = HmiController(sys.argv[1], sys.argv[2], is_calib_enabled = True)
    hmi.start()

    