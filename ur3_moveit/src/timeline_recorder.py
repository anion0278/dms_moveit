#!/usr/bin/env python

import roslaunch
import rospy
import os
import csv
import numpy as np

import config
import util_ros_process as ros_process
import util_common as utils

rospy.init_node('timeline_recorder', anonymous=True)

utils.set_param(config.replan_impulse_param, False)
end = False

max_dist = 0.2 # max distance to be displayed in graph
 
current_script_path = os.path.dirname(os.path.realpath(__file__))

time_step = 0.2

def get_bool(param):
    return int(rospy.get_param(param))

def get_rel_intensivity(param):
    val = rospy.get_param(param)
    return round(float(val) / config.invalid_goal_intensity * 100.0)

def get_rel_distance(param):
    dist = np.clip(rospy.get_param(param), 0, max_dist)
    return round(dist / max_dist * 100, 2)

with open(os.path.join(current_script_path,'timeline.csv'), mode='w') as tl_file:
    writer = csv.writer(tl_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(['time', 'dist_right', 'dist_left', 'dist_obj', 'hmi_val_right', 'hmi_val_left', 'validity', 'replan', "goal_name"])
    try:
        time = 0
        while not end:
            # TODO param names from config
            dist_right = get_rel_distance(config.clearance_param + "_" + config.hmi_right) 
            dist_left = get_rel_distance(config.clearance_param + "_" +  config.hmi_left)
            dist_obj = get_rel_distance(config.clearance_param)
            
            hmi_val_right = get_rel_intensivity(config.notification_val_param + config.hmi_right) 
            hmi_val_left = get_rel_intensivity(config.notification_val_param + config.hmi_left) 

            validity = get_bool(config.goal_validity_param) 

            replan = get_bool(config.replan_param) or get_bool(config.replan_impulse_param)
            utils.set_param(config.replan_impulse_param, False)

            goal_name = utils.get_param(config.goal_name_param)

            writer.writerow([time, dist_right, dist_left, dist_obj, hmi_val_right, hmi_val_left, validity, replan, goal_name])
            rospy.sleep(duration=rospy.Duration(secs=time_step))
            print("Recorded step: %s" % rospy.get_time())

            #time = rospy.get_time()
            time = round(time + time_step, 2) # should not be like that, but...

    except KeyboardInterrupt:
        writer.writerow([time, 0, 0, 0, 0, 0, 0, 0])
        tl_file.close()
        print("Finished recording!")