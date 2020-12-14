#!/usr/bin/env python

import roslaunch
import rospy
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART
import os

import util_ros_process as rp
import config

# no other way to start it correctly 
rospy.init_node('hmi_starter', anonymous=True)
os.system("rosrun ur3_moveit hmi_disconnector.py")

# by starting via rosrun the amout of unrelated info is rediced
rp.run_process_async("rosrun ur3_moveit hmi_controller.py hmi_left _left node")
rospy.wait_for_service(config.hmi_left + config.calibr_service)
rp.run_process_async("rosrun ur3_moveit hmi_controller.py hmi_right _right node")