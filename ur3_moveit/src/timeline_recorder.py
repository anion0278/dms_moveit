#!/usr/bin/env python

import roslaunch
import ros_process
import rospy
import os
import csv
import config
import numpy as np

rospy.init_node('timeline_recorder', anonymous=True)

rospy.set_param("/env_change_impulse", False)
end = False

max_dist = 0.2

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
            dist_right = get_rel_distance("/move_group/collision/min_clearance_right") 
            dist_left = get_rel_distance("/move_group/collision/min_clearance_left")
            dist_obj = get_rel_distance("/move_group/collision/min_clearance")
            
            hmi_val_right = get_rel_intensivity("/debug_hmi_glove_right") 
            hmi_val_left = get_rel_intensivity("/debug_hmi_glove_left") 

            validity = get_bool("/goal_validity") 

            replan = get_bool("/replan") or get_bool("/env_change_impulse")
            rospy.set_param("/env_change_impulse", False)

            goal_name = rospy.get_param("/goal_name")

            writer.writerow([time, dist_right, dist_left, dist_obj, hmi_val_right, hmi_val_left, validity, replan, goal_name])
            rospy.sleep(duration=rospy.Duration(secs=time_step))
            print("Recorded step: %s" % rospy.get_time())

            #time = rospy.get_time()
            time = round(time + time_step, 2) # should not be like that, but...

    except KeyboardInterrupt:
        writer.writerow([time, 0, 0, 0, 0, 0, 0, 0])
        tl_file.close()
        print("Finished recording!")