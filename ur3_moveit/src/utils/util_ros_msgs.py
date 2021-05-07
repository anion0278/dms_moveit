from geometry_msgs.msg import *
import rospy
import numpy as np

def get_stamped_transform(frame_id, trf_frame_id, rotation):
    tfs = TransformStamped(transform = Transform(rotation = rotation))
    tfs.header.stamp = rospy.Time.now()
    tfs.header.frame_id = frame_id
    tfs.child_frame_id = trf_frame_id
    return tfs

def get_identity_quat_arr():
    return [0,0,0,1]

def get_identity_quat_msg():
    return Quaternion(*get_identity_quat_arr())

def get_norm_quaternion_msg(quaternion_data): # quaternions must be normalized, otherwise TF will show errors
    quat_norm = quaternion_data / np.linalg.norm(quaternion_data)
    return Quaternion(*quat_norm)