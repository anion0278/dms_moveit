#!/usr/bin/env python

import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image
import ros_numpy  #sudo apt-get install ros-melodic-ros-numpy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
import numpy as np
from scipy.spatial import distance
import sys
from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped, Transform
import tf2_ros

import task_commander as com
import robot_driver
import config

debug = False


def get_color_range(color_base):
    sensitivity = 15
    upper = (color_base + sensitivity, 255, 255)
    lower = (color_base - sensitivity, 60, 60)
    return (lower, upper)

# hue is in range 0..179 # TODO from config
right_color_range = get_color_range(15)  #red
left_color_range = get_color_range(60)  #green
right_border_color = (0, 0, 255) 
left_border_color = (0, 255, 0)

blob_min_size = 50

camera_name = "camera_top"

dwn_smpl = 8
if debug:
    dwn_smpl = 1

# it is possible to utilize only Depth msg, because it contains RGB data
pc_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('rgb', 12, PointField.UINT32, 1)
]

class HmiTracker:
    def __init__(self):
        self.__driver = robot_driver.RobotDriver(total_speed=0.2)

        self.tf_pub = tf2_ros.TransformBroadcaster()

        pc_topic = "/"+camera_name+"/depth/color/points"
        self.cam_trf = self.get_init_hmi_orientation(pc_topic)
        self.__cv_bridge = CvBridge()
        depth_sub = message_filters.Subscriber(pc_topic, PointCloud2, queue_size=1)

        img_topic = "/"+camera_name+"/color/image_raw"
        camera_sub = message_filters.Subscriber(img_topic, Image, queue_size=1)

        sync = message_filters.ApproximateTimeSynchronizer([depth_sub, camera_sub], queue_size=1, slop=0.05)
        sync.registerCallback(self.on_data)

        tracked_topic_suffix = "_tracked_points"
        self.right_pc_pub = rospy.Publisher(config.hmi_right + tracked_topic_suffix, PointCloud2, queue_size=1)
        self.left_pc_pub = rospy.Publisher(config.hmi_left + tracked_topic_suffix, PointCloud2, queue_size=1)
        self.cam_img_pub = rospy.Publisher("hmi_tracker_image", Image, queue_size=1)

    def run(self):
        rospy.spin()

    def get_init_hmi_orientation(self, pc_topic):
        tf_buf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buf) # required
        cam_frame_id = rospy.wait_for_message(pc_topic, PointCloud2).header.frame_id
        return tf_buf.lookup_transform(cam_frame_id, "world", rospy.Time(), timeout=rospy.Duration(0.1)).transform

    def overlay_circle_on_img(self, image, pos, color):
        cv2.circle(image,
                center=(int(pos[0]), int(pos[1])),
                radius=2,
                color=color,
                thickness=2,
                lineType=8,
                shift=0)

    def on_data(self, depth_msg, img_msg):
        if debug:
            start = rospy.get_time()
            print("Tracker - start time: %s" % start)

        img = self.__cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        final_size = (img.shape[1] / dwn_smpl, img.shape[0] / dwn_smpl)
        img = cv2.resize(img, final_size)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        right_hand = self.find_hand(hsv, depth_msg, right_color_range)
        left_hand = self.find_hand(hsv, depth_msg, left_color_range)

        if right_hand is not None:
            self.display_hand_on_img(img, right_hand, right_border_color)
        if left_hand is not None:
            self.display_hand_on_img(img, left_hand, left_border_color)

        if left_hand is not None or right_hand is not None:
            depth_img = ros_numpy.numpify(depth_msg)

        if (right_hand is not None):
            self.publish_hand(depth_img, depth_msg.header, right_hand, config.hmi_right,
                        self.right_pc_pub, None)
        else:
            self.publish_emtpy_pc(self.right_pc_pub, config.hmi_right, depth_msg.header)
            
        if (left_hand is not None):
            self.publish_hand(depth_img, depth_msg.header, left_hand, config.hmi_left,
                        self.left_pc_pub, None)
        else:
            self.publish_emtpy_pc(self.left_pc_pub, config.hmi_left, depth_msg.header)

        # the topic is not published unless anyone is subcribed
        if (self.cam_img_pub.get_num_connections() > 0):
            img_msg = self.__cv_bridge.cv2_to_imgmsg(img)
            self.cam_img_pub.publish(img_msg)
        pass

        if debug:
            cycle_time = (rospy.get_time() - start) / 1000.0
            print ("Tracker - time spent: %s" % cycle_time)

    def get_hand_pose(self, orient_msg, frame_id, center_pos):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.pose.position.x = center_pos[0]
        p.pose.position.y = center_pos[1]
        p.pose.position.z = center_pos[2]
        p.pose.orientation = Quaternion(0, 0, 0, 1)
        return p


    def publish_hand(self, depth_img,
                    header,
                    hand_tuple,
                    obj_name,
                    pc_pub,
                    orient_msg):

        cv2.drawContours(hand_tuple[3], [hand_tuple[2]],
                            0,
                            color=255,
                            thickness=-1)
        blob_pts = np.where(hand_tuple[3] > 0)

        radius = self.get_bounding_radius(hand_tuple[0], hand_tuple[2])

        # the topic is not published unless anyone is subcribed
        publish_pointcloud = pc_pub.get_num_connections() > 0

        heights = []
        pc_hand = []
        for x, y in zip(blob_pts[0], blob_pts[1]):
            depth_img_point = depth_img[x * dwn_smpl, y * dwn_smpl]
            heights.append(depth_img_point["z"])
            if publish_pointcloud:
                try:
                    pc_hand.append(depth_img_point)
                except:
                    pass # when point is on the edge of image

        if publish_pointcloud:
            cloud_modified = pc2.create_cloud(header, pc_fields, pc_hand)
            pc_pub.publish(cloud_modified)

        cloud_center = self.get_img_coords(depth_img, hand_tuple[0], np.mean(heights))
        poseStamp = self.get_hand_pose(orient_msg, header.frame_id, cloud_center)    
        self.__driver.update_hmi_obj(poseStamp, obj_name, radius)

        # TODO into transform_manager?
        t = TransformStamped(transform = Transform(translation = poseStamp.pose.position, rotation = self.cam_trf.rotation))
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = header.frame_id
        t.child_frame_id = obj_name
        self.tf_pub.sendTransform(t)
        pass

    def is_center_within_contour(self, center, blob):
        return cv2.pointPolygonTest(blob, (center[0], center[1]), True) >= 0

    def find_hand(self, hsv_img, depth_msg, color_range):
        result = None
        mask = cv2.inRange(hsv_img, color_range[0], color_range[1])
        _, contours, _ = cv2.findContours(mask.astype("uint8"), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            blob = max(contours, key=lambda cont: cont.size)
            if blob.size > blob_min_size / dwn_smpl:
                rect = cv2.minAreaRect(blob)
                center = np.array(rect[0]).astype("int")
                if not self.is_center_within_contour(center, blob):
                    # the closest point, otherwise the center's height can be incorreclty calculated
                    center = np.squeeze(min(blob, key=lambda p: distance.euclidean(p, center)))
                result = (center, rect, blob, mask)
        return result

    def publish_emtpy_pc(self, pub, object_name, header):
        self.__driver.remove_hmi_obj(object_name)
        cloud_empty = pc2.create_cloud(header, pc_fields, [])
        pub.publish(cloud_empty)


    def display_hand_on_img(self, img, bound_box, color):
        self.overlay_circle_on_img(img, bound_box[0], color)
        box = cv2.boxPoints(bound_box[1])
        box = np.int0(box) 
        cv2.drawContours(img, [box], 0, color, 2)


    def get_bounding_radius(self, center, blob):
        most_distant_point = max(blob, key=lambda p: distance.euclidean(p, center))
        scale2d = 1450  # magic const
        return distance.euclidean(most_distant_point, center) / scale2d * dwn_smpl


    def get_img_coords(self, depth_img, point, override_height = None):
        x = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["x"]
        y = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["y"]
        z = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["z"]
        if override_height is not None:
            z = override_height
        return (x, y, z)


if __name__ == "__main__":
    t = HmiTracker()
    t.run()