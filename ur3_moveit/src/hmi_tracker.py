#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy  #sudo apt-get install ros-melodic-ros-numpy
import message_filters
from scipy.spatial import distance

import task_commander as com
import robot_driver
import config
import util_common as util
import hmi_tracker_image_processor as ip
import hmi_tracker_cloud_processor as cp
import hmi_tracker_transform_manager as tp

debug = False

class EmptyMoveitInterface():
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
    def __init__(self, camera_name, moveit_interface):
        self.__driver = moveit_interface
        self.epsilon_min_dist_change_m = 0.02
        self.__hmi_cache = {config.hmi_left:0, config.hmi_right:0, config.hmi_left+center_suf:0, config.hmi_right+center_suf:0}

        dwn_smpl = 4
        if debug:
            dwn_smpl = 4
        
        self.img_proc = ip.HmiTrackerImageProcessor(dwn_smpl, debug)
        self.pc_proc = cp.HmiTrackerCloudProcessor(dwn_smpl)
        self.tf_proc = tp.HmiTrackerTransformManager()

        pc_topic = "/"+camera_name+"/depth_registered/points"
        self.tf_proc.init_hmi_orientation(pc_topic)
        self.__init_cam_subs(pc_topic, "/"+camera_name+"/color/image_rect_color") # rect -> rectified image (real camera)

        tracked_topic_suffix = "_tracked_points"
        self.right_pc_pub = rospy.Publisher(config.hmi_right + tracked_topic_suffix, PointCloud2, queue_size=1)
        self.left_pc_pub = rospy.Publisher(config.hmi_left + tracked_topic_suffix, PointCloud2, queue_size=1)

    def __init_cam_subs(self, pc_topic, img_topic):
        depth_sub = message_filters.Subscriber(pc_topic, PointCloud2, queue_size=1)
        camera_sub = message_filters.Subscriber(img_topic, Image, queue_size=1)
        # TODO it is possible to utilize only Depth msg, because it contains RGB data
        sync = message_filters.ApproximateTimeSynchronizer([depth_sub, camera_sub], queue_size=1, slop=0.05)
        sync.registerCallback(self.on_data)

    def run(self):
        rospy.spin()

    def on_data(self, depth_msg, img_msg):
        if debug:
            start = rospy.get_time()
            print("Tracker - start time: %s" % start)

        # its possible to rewrite hands processing into hand-array-processing, but this way its easier to read and debug
        img = self.img_proc.preprocess_img(img_msg)
        left_hand, right_hand = self.img_proc.find_hands(img)
        depth_img = ros_numpy.numpify(depth_msg)
        self.__process_hand_data(right_hand, config.hmi_right, self.right_pc_pub, depth_img, depth_msg)
        self.__process_hand_data(left_hand, config.hmi_left, self.left_pc_pub, depth_img, depth_msg)

        if debug:
            cycle_time = (rospy.get_time() - start) / 1000.0
            print ("Tracker - time spent: %s ms" % cycle_time)

    def __process_hand_data(self, hand_data, hmi_name, pc_pub, depth_img, depth_msg):
        if (hand_data is not None):
            self.publish_hand(depth_img, depth_msg.header, hand_data, hmi_name,pc_pub)
        else:
            self.publish_empty_data(hmi_name, pc_pub, depth_msg.header)

    def publish_empty_data(self, hmi_name, pc_pub, header):
        self.__driver.remove_hmi_obj(hmi_name)
        self.pc_proc.publish_emtpy_pc(pc_pub, header)
        zero_pose = self.tf_proc.get_hand_pose(header.frame_id, [0,0,0])  
        self.tf_proc.publish_hmi_tf(zero_pose, "non_existent_frame", hmi_name)
        self.__hmi_cache[hmi_name] = 0
        self.__hmi_cache[hmi_name+center_suf] = 0

    def publish_hand(self, depth_img,header,hand_data,hmi_name,pc_pub):
        cloud_center, radius = self.pc_proc.get_center_and_publish(pc_pub, hand_data, depth_img, header)
        
        if cloud_center is not None:
            # TF should be published anyway, because it decays fast !!!
            stamped_pose = self.tf_proc.get_hand_pose(header.frame_id, cloud_center)   
            self.tf_proc.publish_hmi_tf(stamped_pose, header.frame_id, hmi_name)
            
            if abs(self.__hmi_cache[hmi_name] - radius) > self.epsilon_min_dist_change_m or \
                distance.euclidean(self.__hmi_cache[hmi_name+center_suf], cloud_center) > self.epsilon_min_dist_change_m: 
                # the change of the radius or position was large enough to be noticable

                # objs = self.__driver.scene.get_known_object_names()
                # if  obj_name in objs:
                #     self.__driver.move_hmi_obj(stamped_pose, obj_name, radius)
                # else:
                #     self.__driver.add_hmi_obj(stamped_pose, obj_name, radius)
                
                self.__driver.add_hmi_obj(stamped_pose, hmi_name, radius)
                self.__hmi_cache[hmi_name] = radius
                self.__hmi_cache[hmi_name+center_suf] = cloud_center


if __name__ == "__main__":
    
    sys.argv.append("moveit")
    if "moveit" in sys.argv: 
        print("Tracker: MOVEIT MODE")
        moveit_interface = robot_driver.RobotDriver(total_speed=0.2)
    else:
        print("Tracker: HMI DEMO MODE!")
        moveit_interface = EmptyMoveitInterface()

    if debug: util.print_all_args()

    t = HmiTracker("camera_top", moveit_interface)
    t.run()