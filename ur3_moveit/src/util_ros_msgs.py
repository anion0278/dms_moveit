from geometry_msgs.msg import *
import rospy

def get_stamped_transform(frame_id, trf_frame_id, rotation):
    tfs = TransformStamped(transform = Transform(rotation = rotation))
    tfs.header.stamp = rospy.Time.now()
    tfs.header.frame_id = frame_id
    tfs.child_frame_id = trf_frame_id
    return tfs

def get_identity_quat_arr():
    return [0,0,0,1]

def get_identity_quat():
    return Quaternion(*get_identity_quat_arr())