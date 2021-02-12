#!/usr/bin/env python

import rospy
import os
import sys
import setproctitle
setproctitle.setproctitle("DMS HMI Starter")

import util_common as util
import util_ros_process as rp
import config

def wait_for_hmi(hmi_name):
    print("Waiting for: %s" % hmi_name)
    rospy.wait_for_service(hmi_name + config.calibr_service)
    print("Got response from: %s" % hmi_name)

def start_and_wait_for_hmi(hmi_name, args):
    rp.run_process_async("rosrun ur3_moveit hmi_controller.py device_name:=%s mode:=node %s" % (hmi_name, args))
    wait_for_hmi(hmi_name)

def connect_both_hmi(args):
    rospy.init_node("hmi_starter")
    # there is no other way to start it correctly!
    os.system("rosrun ur3_moveit hmi_controller_disconnector.py")
    # by starting via rosrun the amout of unrelated info is rediced
    start_and_wait_for_hmi(config.hmi_left, args)
    start_and_wait_for_hmi(config.hmi_right, args)
    rp.run_process_async("rosrun ur3_moveit task_heartbeat_watchdog.py")

if __name__ == "__main__":
    util.print_all_args()
    connect_both_hmi(sys.argv[1])