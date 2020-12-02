#!/usr/bin/env python

import roslaunch
import rospy
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
import os

import util_ros_process as rp

# no other way to start it correctly 
rospy.init_node('hmi_starter', anonymous=True)
os.system("rosrun ur3_moveit hmi_disconnector.py")

rp.run_process_async("roslaunch ur3_moveit left_hmi.launch")
rospy.wait_for_service("hmi_glove_left_service")
rp.run_process_async("roslaunch ur3_moveit right_hmi.launch")