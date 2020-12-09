import os

from enum import Enum
class Side(Enum):
    LEFT = 1
    RIGHT = 2

hmi_right = "hmi_right"
hmi_left = "hmi_left"

status_replan = "replan"
status_invalid = "invalid"

invalid_goal_intensity = 170
replan_intensity = 130
dist_intensity_max = 100
dist_intensity_min = 60
vibr_min = 60

color_left = [0, 1, 0] 
color_right = [1, 0, 0]

calibr_service = "_frame_calibration"
offsets_file = "{0}_imu_offsets.p"
calibr_file = "{0}_frame_calibration.p"


current_script_path = os.path.dirname(os.path.realpath(__file__))

def get_file_full_path(file_name):
    return os.path.join(current_script_path, file_name)

def get_offsets_file_path(node_name):
    return get_file_full_path(offsets_file.format(node_name))

def get_frame_calibration_file_path(node_name):
    return get_file_full_path(calibr_file.format(node_name))