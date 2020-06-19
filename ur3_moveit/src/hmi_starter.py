#!/usr/bin/env python

import roslaunch
import ros_process
import rospy
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
import os

# just could not do it any other way
# tried all possibilities

rospy.init_node('hmi_starter', anonymous=True)
os.system("rosrun ur3_moveit hmi_disconnector.py")

ros_process.run_process_async("roslaunch ur3_moveit left_hmi.launch")
rospy.wait_for_service("hmi_glove_left_service")
ros_process.run_process_async("roslaunch ur3_moveit right_hmi.launch")