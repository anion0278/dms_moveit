
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image
import ros_numpy #sudo apt-get install ros-melodic-ros-numpy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters

debug = True

sensitivity = 20
hsv_color_base = 240
upper = (hsv_color_base+sensitivity,255,255)  
lower = (hsv_color_base-sensitivity,50,50)

dwn_smpl = 8

pc_fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),]

def overlay_circle_on_img(image, pos, color=(255,0,0)):
    cv2.circle(image, center = (int(pos[0]),int(pos[1])), radius = 2,  color = color, thickness=2, lineType=8, shift=0) 

def on_data(depth_msg, img_msg):
    img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    final_size = (img.shape[1] / dwn_smpl, img.shape[0] / dwn_smpl)
    img = cv2.resize(img, final_size)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img, lower, upper)
    _, contours,_ = cv2.findContours(mask.astype("uint8"), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        blob = max(contours, key=lambda cont: cont.size)
        M = cv2.moments(blob)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) 

        # ~ 2x faster, less accurate
        #rect = cv2.boundingRect(blob)
        #center = (rect[0] + rect[2] / 2, rect[1] + rect[3] / 2)

        overlay_circle_on_img(img, center)
        depth_img = ros_numpy.numpify(depth_msg)

        # simple, naive center
        z = depth_img[center[1] * dwn_smpl, center[0] * dwn_smpl]["z"]
        print("Center height: %s", z)

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

            cloud_modified = pc2.create_cloud(depth_msg.header, pc_fields, pc_hand)
            cloud_pub.publish(cloud_modified)
            print("Precise height: %s", z)

        cv2.imshow('image', mask)
        cv2.waitKey(1)

if __name__ == "__main__":
    bridge = CvBridge()
    rospy.init_node('top_cam_handler')
    depth_sub = message_filters.Subscriber("/camera_top/depth/color/points", PointCloud2, queue_size = 1)
    camera_sub = message_filters.Subscriber("/camera_top/color/image_raw", Image, queue_size = 1)
    sync = message_filters.ApproximateTimeSynchronizer([depth_sub, camera_sub], 1, slop = 0.05) # also queue size
    sync.registerCallback(on_data)

    cloud_pub = rospy.Publisher("right_hmi_points", PointCloud2, queue_size=1)

    rospy.spin()