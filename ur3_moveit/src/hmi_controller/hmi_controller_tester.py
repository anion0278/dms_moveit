#!/usr/bin/env python
import rospy
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
import setproctitle
setproctitle.setproctitle("DMS HMI Tester")

import sys, os
sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )
from config import config
from config.config import TaskStatus
from utils import util_common as utils
from utils import util_ros_msgs

import hmi_controller_starter

utils.set_param(config.task_status_param, TaskStatus.OK.value)

hmi_controller_starter.connect_both_hmi("directed_vibration:=true")

def get_arrow(ns, id, end_point):
    m = Marker(type = Marker.ARROW, 
                    pose = Pose(orientation = util_ros_msgs.get_identity_quat_msg()), 
                    action = Marker.ADD, 
                    lifetime = config.tf_viz_decay_duration_s,
                    scale = Point(0.1, 0.1, 0.1), 
                    color = ColorRGBA(1,1,1,1), 
                    points = [end_point, Point()],                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
                    ns = ns, id = id)
    m.header.frame_id = "world"
    return m

def set_hmi_tf(hmi_name):
    t = TransformStamped(transform = Transform(rotation = util_ros_msgs.get_identity_quat_msg()))
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = hmi_name
    tf_pub.sendTransform(t)

tf_pub = tf2_ros.StaticTransformBroadcaster()
hmi_collision_vector_pub = rospy.Publisher(config.collision_vec_topic, MarkerArray, queue_size=2)

rate = rospy.Rate(10)
while not rospy.is_shutdown():  
    rate.sleep()
    set_hmi_tf(config.hmi_left)
    set_hmi_tf(config.hmi_right)
    utils.set_param(config.task_status_param, TaskStatus.OK.value) #TaskStatus.OK.value
 
    dist_to_hmi = 0.01
    left = get_arrow(config.hmi_left+"_vector", 1, Point(dist_to_hmi,0,0))
    right = get_arrow(config.hmi_right+"_vector", 2, Point(dist_to_hmi,0,0))
    ma = MarkerArray(markers = [left, right])
    hmi_collision_vector_pub.publish(ma)

    utils.set_param(config.clearance_param + "_" + config.hmi_left, dist_to_hmi)
    utils.set_param(config.clearance_param + "_" + config.hmi_right, dist_to_hmi)
    