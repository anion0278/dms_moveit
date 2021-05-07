#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy  #sudo apt-get install ros-melodic-ros-numpy
import message_filters
from scipy.spatial import distance
import setproctitle
setproctitle.setproctitle("DMS HMI Tracker")

import sys, os
sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )
from task import robot_driver
from config import config
from utils import util_common as util
import hmi_tracker_image_processor as ip
import hmi_tracker_cloud_processor as cp
import hmi_tracker_transform_manager as tp

debug = False

class EmptyMoveitInterface(): # for HMI demo purposes
    def __init__(self):
        rospy.init_node('robot_python_driver', anonymous=True)
    
    def add_hmi_obj(self, pose, name, radius): 
        pass 

    def move_hmi_obj(self, pose, name, radius):
        pass

    def remove_hmi_obj(self, name):
        pass

center_suf = "_center"

class HmiTracker:
    def __init__(self, camera_names, main_camera_name, moveit_interface):
        self.__driver = moveit_interface
        self.epsilon_min_dist_change_m = 0.01
        self.__hmi_cache = {config.hmi_left:0, config.hmi_right:0, config.hmi_left+center_suf:0, config.hmi_right+center_suf:0}
        self.main_camera_name = main_camera_name
        self.main_camera_frame_id = self.main_camera_name+"_color_optical_frame"

        dwn_smpl = 4
        if debug:
            dwn_smpl = 1
        
        self.img_proc = ip.HmiTrackerImageProcessor(dwn_smpl, debug)
        self.pc_proc = cp.HmiTrackerCloudProcessor(dwn_smpl)
        self.tf_proc = tp.HmiTrackerTransformManager()
        
        self.__init_cam_subs(camera_names)

        tracked_topic_suffix = "_tracked_points"
        self.right_pc_pub = rospy.Publisher(config.hmi_right + tracked_topic_suffix, PointCloud2, queue_size=1)
        self.left_pc_pub = rospy.Publisher(config.hmi_left + tracked_topic_suffix, PointCloud2, queue_size=1)

    def __init_cam_subs(self, camera_names):
        if len(camera_names) == 0: ValueError("Check camera names!")
        subs = []
        for camera_name in camera_names:
            pc_topic = "/"+camera_name+"/depth_registered/points"
            if self.main_camera_name == camera_name:
                self.tf_proc.init_hmi_orientation(pc_topic)
            subs.append(message_filters.Subscriber(pc_topic, PointCloud2, queue_size=1))
            subs.append(message_filters.Subscriber("/"+camera_name+"/color/image_rect_color", Image, queue_size=1))
        sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=1, slop=0.05)
        sync.registerCallback(self.on_data)

    def run(self):
        rospy.spin()

    def on_data(self, *ros_msgs):
        if debug:
            start = rospy.get_time()
            print("Tracker - start time: %s" % start)
        
        self.msg_processing(ros_msgs)

        if debug:
            cycle_time = (rospy.get_time() - start)
            print ("Tracker - time spent: %s s" % cycle_time)

    def msg_processing(self, ros_msgs):
        pc_left_hand = np.empty((0, 3))
        pc_right_hand = np.empty((0, 3))

        for i in range(0, len(ros_msgs), 2):
            pc_msg = ros_msgs[i]
            img_msg = ros_msgs[i+1]
            
            img = self.img_proc.preprocess_img(img_msg)
            left_hand, right_hand, tracked_img = self.img_proc.find_hands(img)

            if self.main_camera_frame_id == pc_msg.header.frame_id:
                self.img_proc.publish_img_if_required(tracked_img)

            if left_hand is not None:
                pc_left_hand = self.__process_hand_data(left_hand, pc_msg, pc_left_hand)

            if right_hand is not None:
                pc_right_hand = self.__process_hand_data(right_hand, pc_msg, pc_right_hand)
        
        self.__publish_data(config.hmi_left, self.left_pc_pub, pc_left_hand, self.main_camera_frame_id)
        self.__publish_data(config.hmi_right, self.right_pc_pub, pc_right_hand, self.main_camera_frame_id)
        pass

    def __process_hand_data(self, hand, pc_msg, pc_hand):
        pc_r = self.pc_proc.get_hmi_point_cloud(hand, pc_msg)
        pc_r_tr = self.tf_proc.transform_pc(pc_r, self.main_camera_frame_id, pc_msg.header.frame_id)
        if len(pc_r_tr) > 0:
            pc_hand = np.append(pc_hand, pc_r_tr, axis=0)
        return pc_hand

    def __publish_data(self, hmi_name, pc_pub, pc_hand, frame_id):
        filtered_cloud_o3d = self.pc_proc.filter_point_cloud(pc_hand)
        if len(filtered_cloud_o3d.points) == 0:
            self.publish_empty_data(hmi_name, pc_pub, frame_id)
        else:
            self.publish_hand(filtered_cloud_o3d, hmi_name, pc_pub, frame_id)

    def publish_empty_data(self, hmi_name, pc_pub, frame_id):
        self.__driver.remove_hmi_obj(hmi_name)
        self.pc_proc.publish_emtpy_pc(pc_pub, frame_id)
        zero_pose = self.tf_proc.get_hand_pose(frame_id, [0,0,0])  
        self.tf_proc.publish_hmi_tf(zero_pose, frame_id, hmi_name)
        self.__hmi_cache[hmi_name] = 0
        self.__hmi_cache[hmi_name+center_suf] = 0

    def publish_hand(self, filtered_cloud_o3d, hmi_name, pc_pub, frame_id):
        sphere_center, sphere_radius, ros_pc = self.pc_proc.get_pc_bounding_sphere(filtered_cloud_o3d, frame_id)
        self.pc_proc.publish_if_required(pc_pub, ros_pc)

        # TF should be published anyway, because it decays fast !!!
        stamped_pose = self.tf_proc.get_hand_pose(frame_id, sphere_center)   
        self.tf_proc.publish_hmi_tf(stamped_pose, frame_id, hmi_name)
        
        if abs(self.__hmi_cache[hmi_name] - sphere_radius) > self.epsilon_min_dist_change_m or \
            distance.euclidean(self.__hmi_cache[hmi_name+center_suf], sphere_center) > self.epsilon_min_dist_change_m: 
            # the change of the radius or position was large enough to be noticable
            self.__driver.add_hmi_obj(stamped_pose, hmi_name, sphere_radius)
            self.__hmi_cache[hmi_name] = sphere_radius
            self.__hmi_cache[hmi_name+center_suf] = sphere_center


if __name__ == "__main__":

    sys.argv.append("moveit")
    if "moveit" in sys.argv: 
        print("Tracker: MOVEIT MODE")
        moveit_interface = robot_driver.RobotDriver(total_speed=0.2)
    else:
        print("Tracker: HMI DEMO MODE!")
        moveit_interface = EmptyMoveitInterface()

    util.print_all_args()

    t = HmiTracker(["camera_top", "camera_left", "camera_right"], "camera_top", moveit_interface)
    t.run()