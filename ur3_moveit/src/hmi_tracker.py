#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy  #sudo apt-get install ros-melodic-ros-numpy
import message_filters

import task_commander as com
import robot_driver
import config
import hmi_tracker_image_processor as ip
import hmi_tracker_cloud_processor as cp
import hmi_tracker_transform_manager as tp

debug = True

class HmiTracker:
    def __init__(self, camera_name):
        self.__driver = robot_driver.RobotDriver(total_speed=0.2)

        dwn_smpl = 8
        if debug:
            dwn_smpl = 1
        
        self.img_proc = ip.HmiTrackerImageProcessor(dwn_smpl, debug)
        self.pc_proc = cp.HmiTrackerCloudProcessor(dwn_smpl)
        self.tf_proc = tp.HmiTrackerTransformManager()

        pc_topic = "/"+camera_name+"/depth/color/points"
        self.tf_proc.init_hmi_orientation(pc_topic)
        self.__init_cam_subs(pc_topic, "/"+camera_name+"/color/image_raw")

        tracked_topic_suffix = "_tracked_points"
        self.right_pc_pub = rospy.Publisher(config.hmi_right + tracked_topic_suffix, PointCloud2, queue_size=1)
        self.left_pc_pub = rospy.Publisher(config.hmi_left + tracked_topic_suffix, PointCloud2, queue_size=1)

    def __init_cam_subs(self, pc_topic, img_topic):
        depth_sub = message_filters.Subscriber(pc_topic, PointCloud2, queue_size=1)
        camera_sub = message_filters.Subscriber(img_topic, Image, queue_size=1)
        sync = message_filters.ApproximateTimeSynchronizer([depth_sub, camera_sub], queue_size=1, slop=0.05)
        sync.registerCallback(self.on_data)

    def run(self):
        rospy.spin()

    def on_data(self, depth_msg, img_msg):
        if debug:
            start = rospy.get_time()
            print("Tracker - start time: %s" % start)

        img = self.img_proc.preprocess_img(img_msg)

        left_hand, right_hand = self.img_proc.find_hands(img)
        if left_hand is not None or right_hand is not None:
            depth_img = ros_numpy.numpify(depth_msg)
            self.__process_hand_data(right_hand, config.hmi_right, self.right_pc_pub, depth_img, depth_msg)
            self.__process_hand_data(left_hand, config.hmi_left, self.left_pc_pub, depth_img, depth_msg)

        if debug:
            cycle_time = (rospy.get_time() - start) / 1000.0
            print ("Tracker - time spent: %s" % cycle_time)

    def __process_hand_data(self, hand_data, hmi_name, pc_pub, depth_img, depth_msg):
        if (hand_data is not None):
            self.publish_hand(depth_img, depth_msg.header, hand_data, hmi_name,pc_pub)
        else:
            self.publish_empty_data(hmi_name, pc_pub, depth_msg.header)

    def publish_empty_data(self, hmi_name, pc_pub, header):
        self.__driver.remove_hmi_obj(hmi_name)
        self.pc_proc.publish_emtpy_pc(pc_pub, header)

    def publish_hand(self, depth_img,header,hand_tuple,obj_name,pc_pub):
        radius = self.img_proc.get_bounding_radius(hand_tuple)
        cloud_center = self.pc_proc.get_center_and_publish(pc_pub, hand_tuple, depth_img, header)
        stamped_pose = self.tf_proc.get_hand_pose(header.frame_id, cloud_center)    
        self.__driver.update_hmi_obj(stamped_pose, obj_name, radius)
        self.tf_proc.publish_hmi_tf(stamped_pose, header.frame_id, obj_name)


if __name__ == "__main__":
    t = HmiTracker(camera_name = "camera_top")
    t.run()