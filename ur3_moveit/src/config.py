import os

from enum import Enum
class Side(Enum):
    LEFT = 1
    RIGHT = 2

hmi_right = "hmi_right"
hmi_left = "hmi_left"

status_replan = "replan"
status_invalid = "invalid"

invalid_goal_intensity = 250
replan_intensity = 220
dist_intensity_max = 200
dist_intensity_min = 150
vibr_min = 60

# invalid_goal_intensity = 170
# replan_intensity = 130
# dist_intensity_max = 100
# dist_intensity_min = 60
# vibr_min = 60

color_left = [0, 1, 0] 
color_right = [1, 0, 0]

calibr_service = "_frame_calibration"
offsets_file = "{0}_imu_offsets.p"
calibr_file = "{0}_frame_calibration.p"

collision_vec_topic = "/move_group/collision_vectors"

current_script_path = os.path.dirname(os.path.realpath(__file__))

def get_file_full_path(file_name):
    return os.path.join(current_script_path, file_name)

def get_offsets_file_path(node_name):
    return get_file_full_path(offsets_file.format(node_name))

def get_frame_calibration_file_path(node_name):
    return get_file_full_path(calibr_file.format(node_name))

def get_visual_color(device_name):
    if "left" in device_name:
        return color_left
    if "right" in device_name:
        return color_right
    raise AttributeError

def get_rviz_color(device_name):
    c = get_visual_color(device_name)
    c.append(0.7) # alpha
    return c

def get_second_hmi_name(first_hmi_name):
    left = "left"
    right = "right"
    if left in first_hmi_name:
        return first_hmi_name.replace(left, right)
    if right in first_hmi_name:
        return first_hmi_name.replace(right, left)
    raise AttributeError("Check hand name!")

def get_side(hmi_name):
    return hmi_name.replace("hmi_", "")