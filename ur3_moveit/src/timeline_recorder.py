#!/usr/bin/env python

import roslaunch
import ros_process
import rospy
import os
import csv
import config

rospy.init_node('timeline_recorder', anonymous=True)
end = False

max_dist = 0.2

current_script_path = os.path.dirname(os.path.realpath(__file__))

time = 0

time_step = 0.2

with open(os.path.join(current_script_path,'timeline.csv'), mode='w') as tl_file:
    writer = csv.writer(tl_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(['time', 'dist_right', 'dist_left', 'dist_obj', 'hmi_val_right', 'hmi_val_left'])
    while not end:
        #time = rospy.get_time()
        time = round(time + time_step, 2) # should not be like that, but...
        dist_right = round(rospy.get_param("/move_group/collision/min_clearance_right") / max_dist * 100, 2)
        dist_left = round(rospy.get_param("/move_group/collision/min_clearance_left") / max_dist * 100, 2)
        dist_obj = round(rospy.get_param("/move_group/collision/min_clearance") / max_dist * 100, 2)
        hmi_val_right = round(rospy.get_param("/debug_hmi_glove_right") / 255 * 100, 2)
        hmi_val_left = round(rospy.get_param("/debug_hmi_glove_left") / 255 * 100, 2)
        writer.writerow([time, dist_right, dist_left, dist_obj, hmi_val_right, hmi_val_left])
        rospy.sleep(duration=rospy.Duration(secs=time_step))
