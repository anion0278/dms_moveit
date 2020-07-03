#!/usr/bin/env python

import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image
import ros_numpy  #sudo apt-get install ros-melodic-ros-numpy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
import robot_driver
import numpy as np
from scipy.spatial import distance
import custom_commander as com
import sys

debug = True

def get_color_range(color_base):
    sensitivity = 15
    upper = (color_base + sensitivity, 255, 255)
    lower = (color_base - sensitivity, 60, 60)
    return (lower, upper)

# hue is in range 0..179
right_color_range = get_color_range(60)  #green
left_color_range = get_color_range(15)  #red

right_border_color = (0, 255, 0)
left_border_color = (0, 0, 255)

blob_min_size = 50

left_hmi = "hmi_left"
right_hmi = "hmi_right"

camera_name = "camera_top"

dwn_smpl = 8
if debug:
    dwn_smpl = 1

pc_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('rgb', 12, PointField.UINT32, 1)
]


def overlay_circle_on_img(image, pos, color):
    cv2.circle(image,
               center=(int(pos[0]), int(pos[1])),
               radius=2,
               color=color,
               thickness=2,
               lineType=8,
               shift=0)

def on_data(depth_msg, img_msg):
    img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    final_size = (img.shape[1] / dwn_smpl, img.shape[0] / dwn_smpl)
    img = cv2.resize(img, final_size)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    right_hand = find_hand(hsv, depth_msg, right_color_range)
    left_hand = find_hand(hsv, depth_msg, left_color_range)

    if right_hand is not None:
        display_hand_on_img(img, right_hand, right_border_color)
    if left_hand is not None:
        display_hand_on_img(img, left_hand, left_border_color)

    if left_hand is not None or right_hand is not None:
        depth_img = ros_numpy.numpify(depth_msg)

    if (right_hand is not None):
        publish_hand(depth_img, depth_msg.header, right_hand, right_hmi,
                     right_pc_pub, debug)
    else:
        driver.remove_hmi_obj(right_hmi)
        publish_emtpy_pc(right_pc_pub, depth_msg.header)
    if (left_hand is not None):
        publish_hand(depth_img, depth_msg.header, left_hand, left_hmi,
                     left_pc_pub, debug)
    else:
        driver.remove_hmi_obj(left_hmi)
        publish_emtpy_pc(left_pc_pub, depth_msg.header)

    cv2.imshow('image', img)
    cv2.waitKey(1)


def publish_hand(depth_img,
                 header,
                 hand_tuple,
                 obj_name,
                 pc_pub,
                 publish_pointcloud=False):

    cv2.drawContours(hand_tuple[3], [hand_tuple[2]],
                        0,
                        color=255,
                        thickness=-1)
    blob_pts = np.where(hand_tuple[3] > 0)

    radius = get_bounding_radius(hand_tuple[0], hand_tuple[2])

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

    cloud_center = get_img_coords(depth_img, hand_tuple[0], np.mean(heights))

    driver.update_hmi_obj(cloud_center, header.frame_id, obj_name, radius)

def is_center_within_contour(center, blob):
    return cv2.pointPolygonTest(blob, (center[0], center[1]), True) >= 0

def find_hand(hsv_img, depth_msg, color_range):
    result = None
    mask = cv2.inRange(hsv_img, color_range[0], color_range[1])
    _, contours, _ = cv2.findContours(mask.astype("uint8"), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        blob = max(contours, key=lambda cont: cont.size)
        if blob.size > blob_min_size / dwn_smpl:
            rect = cv2.minAreaRect(blob)
            center = np.array(rect[0]).astype("int")
            if not is_center_within_contour(center, blob):
                # the closest point, otherwise the center's height can be incorreclty calculated
                center = np.squeeze(min(blob, key=lambda p: distance.euclidean(p, center)))
            result = (center, rect, blob, mask)
    return result

def publish_emtpy_pc(pub, header):
    cloud_empty = pc2.create_cloud(header, pc_fields, [])
    pub.publish(cloud_empty)


def display_hand_on_img(img, bound_box, color):
    overlay_circle_on_img(img, bound_box[0], color)
    box = cv2.boxPoints(bound_box[1])
    box = np.int0(box)  # what it does?
    cv2.drawContours(img, [box], 0, color, 2)


def get_bounding_radius(center, blob):
    most_distant_point = max(blob, key=lambda p: distance.euclidean(p, center))
    scale2d = 1450  # magic const
    return distance.euclidean(most_distant_point, center) / scale2d * dwn_smpl


def get_img_coords(depth_img, point, override_height = None):
    x = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["x"]
    y = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["y"]
    z = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["z"]
    if override_height is not None:
        z = override_height
    return (x, y, z)


if __name__ == "__main__":
    driver = robot_driver.RobotDriver(total_speed=0.2)

    bridge = CvBridge()
    # rospy.init_node('top_cam_handler')
    depth_sub = message_filters.Subscriber("/"+camera_name+"/depth/color/points",
                                           PointCloud2,
                                           queue_size=1)
    camera_sub = message_filters.Subscriber("/"+camera_name+"/color/image_raw",
                                            Image,
                                            queue_size=1)
    sync = message_filters.ApproximateTimeSynchronizer([depth_sub, camera_sub],
                                                       queue_size=1,
                                                       slop=0.05)
    sync.registerCallback(on_data)

    right_pc_pub = rospy.Publisher(right_hmi + "_points",
                                   PointCloud2,
                                   queue_size=1)
    left_pc_pub = rospy.Publisher(left_hmi + "_points",
                                  PointCloud2,
                                  queue_size=1)

    rospy.spin()