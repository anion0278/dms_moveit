import os
import numpy as np
import rospy

from enum import Enum
class Side(Enum):
    LEFT = 1
    RIGHT = 2

class TaskStatus(Enum):
    OK = 0
    INTERRUPTED = 1
    INVALID_GOAL = 2

hmi_right = "hmi_right"
hmi_left = "hmi_left"

# Param Server Names
task_status_param = "task_status"
clearance_param = "/move_group/collision/min_clearance"
plan_interrupted_param = "/replan"
replan_impulse_param = "/env_change_impulse"
goal_name_param = "/goal_name"
goal_validity_param = "/goal_validity"
notification_val_param =  "/debug_"


collision_vec_topic = "/move_group/collision_vectors"

invalid_goal_intensity = 180 # replan stronger than invalid goal, because is more frequents
replan_intensity = 200
dist_intensity_max = 160
dist_intensity_min = 120
vibr_min = 60
vibr_max = 255

tf_viz_decay_duration_s = rospy.Duration(0.2)

color_left = np.array([0, 1, 0]) 
color_right = np.array([1, 0, 0])

calibr_service = "_frame_calibration"
offsets_file = "{0}_imu_offsets.p"
calibr_file = "{0}_frame_calibration.p"

current_script_path = os.path.dirname(os.path.realpath(__file__))

calibration_folder = "hmi_calibration"

def get_file_full_path(file_name, folder = ""):
    return os.path.join(current_script_path, folder, file_name)

def get_offsets_file_path(node_name):
    return get_file_full_path(offsets_file.format(node_name), calibration_folder)

def get_frame_calibration_file_path(node_name):
    return get_file_full_path(calibr_file.format(node_name), calibration_folder)

def get_visual_color(device_name):
    if "left" in device_name:
        return color_left
    if "right" in device_name:
        return color_right
    raise AttributeError

def get_rviz_color(device_name):
    c = get_visual_color(device_name)
    return np.append(c, 0.7) # alpha

    