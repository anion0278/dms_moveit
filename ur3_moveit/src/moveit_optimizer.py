#!/usr/bin/python
from pyswarm import pso
import os
import sys
import subprocess
import ros_process
import rospy
import time

current_script_path = os.path.dirname(os.path.realpath(__file__))


def opti_function(params):
    segment_fraction = params[0]
    octomap_resolution = params[1]
    point_subsample = params[2]
    return segment_fraction + octomap_resolution - point_subsample


lb = [-3, -1, 0]
ub = [2, 6, 2]

xopt, fopt = pso(opti_function, lb, ub)
print (xopt)

sim_status_param = "sim_ok"
max_attempts = 2
if __name__ == "__main__":

    # -points_subsample={0} -octomap_resolution={1} -segment_fraction={2} -repetitions={3}".format(1, 2, 3, 10))

    ros_process.close_roscore()

    for index in range(500):
        is_cycle_ok = False
        attempts = 0
        while not is_cycle_ok and attempts < max_attempts:
            attempts += 1
            print("Starting roscore...")
            ros_process.start_roscore()
            rospy.set_param(sim_status_param, False)
            print("Starting process...")
            ros_process.run_process_sync("roslaunch ur3_moveit start_meas.launch")
            print("Cycle: %s" % index)
            is_cycle_ok = rospy.get_param(sim_status_param)
            print("killing roscore")
            ros_process.close_roscore()
            while ros_process.is_roscore_running():
                time.sleep(1)
            time.sleep(4)
            print("Cycle end")
        print("Attempts %s" % attempts)
        if attempts >= max_attempts:
            raise Exception("Problem with gazebo")
