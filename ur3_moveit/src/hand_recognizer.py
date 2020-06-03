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

debug = True

driver = robot_driver.RobotDriver(total_speed=0.2)

sensitivity = 20
right_color_base = 240
right_upper = (right_color_base + sensitivity, 255, 255)
right_lower = (right_color_base - sensitivity, 50, 50)

hmi_right_color_range = (right_lower, right_upper)

blob_min_size = 50

dwn_smpl = 8
if debug:
    dwn_smpl = 1

pc_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('rgb', 12, PointField.UINT32, 1)
    ]


def overlay_circle_on_img(image, pos, color=(255, 0, 0)):
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
    mask = cv2.inRange(img, hmi_right_color_range[0], hmi_right_color_range[1])
    _, contours, _ = cv2.findContours(mask.astype("uint8"), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        blob = max(contours, key=lambda cont: cont.size)

        if blob.size > blob_min_size / dwn_smpl:

            rect = cv2.minAreaRect(blob)
            box = cv2.boxPoints(rect) 
            box = np.int0(box)
            cv2.drawContours(img,[box],0,(255,0,0),2)

            center = np.array(rect[0]).astype("int")

            overlay_circle_on_img(img, center)
            depth_img = ros_numpy.numpify(depth_msg)

            # simple, naive center
            cloud_center=get_img_coords(depth_img, center)
            print("Center height: %s", cloud_center[2])
            
            most_distant_point = max(blob, key=lambda p: distance.euclidean(p, center))

            overlay_circle_on_img(img, most_distant_point[0])

            box_size = get_bounding_box_size(depth_img, center, most_distant_point[0])
            driver.create_hmi_obj(cloud_center,"hmi_right", box_size / 2)
        
            if debug:
                cv2.drawContours(mask, [blob], 0, color=255, thickness=-1)
                blob_pts = np.where(mask > 0)

                z_heights = []
                pc_hand = []
                for x, y in zip(blob_pts[0], blob_pts[1]):
                    try:
                        depth_img_point = depth_img[x * dwn_smpl, y * dwn_smpl]
                        pc_hand.append(depth_img_point)
                        z_heights.append(depth_img_point["z"])
                    except:
                        pass
                z = np.mean(np.array(z_heights).astype(np.float))

                cloud_modified = pc2.create_cloud(depth_msg.header, pc_fields,
                                                pc_hand)
                cloud_pub.publish(cloud_modified)
                print("Precise height: %s", z)

        cv2.imshow('image', img)
        cv2.waitKey(1)

def get_bounding_box_size(depth_img, center, most_dist):
    corner_1 = get_img_coords(depth_img, center)[:-1]       
    #corner_2 = get_img_coords(depth_img, box_coords[1])[:-1]        
    corner_3 = get_img_coords(depth_img, most_dist)[:-1]  
    width = distance.euclidean(corner_1, corner_3) 
    #height = distance.euclidean(corner_2, corner_3) 
    #length = 0.1
    return width    


def get_img_coords(depth_img, point):
    x = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["x"]
    y = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["y"]
    z = depth_img[point[1] * dwn_smpl, point[0] * dwn_smpl]["z"]
    return (x,y,z)


if __name__ == "__main__":
    bridge = CvBridge()
   # rospy.init_node('top_cam_handler')
    depth_sub = message_filters.Subscriber("/camera_top/depth/color/points",
                                           PointCloud2,
                                           queue_size=1)
    camera_sub = message_filters.Subscriber("/camera_top/color/image_raw",
                                            Image,
                                            queue_size=1)
    sync = message_filters.ApproximateTimeSynchronizer(
        [depth_sub, camera_sub], 1, slop=0.05)  # also queue size
    sync.registerCallback(on_data)

    cloud_pub = rospy.Publisher("right_hmi_points", PointCloud2, queue_size=1)

    rospy.spin()