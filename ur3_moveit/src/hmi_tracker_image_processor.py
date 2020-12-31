import cv2
from scipy.spatial import distance
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import config

class HmiTrackerImageProcessor:
    def __init__(self, dwn_smpl, debug):
        # hue is in range 0..179 # TODO from config
        self.right_color_range = self.get_hue_color_range(15)  #red
        self.left_color_range = self.get_hue_color_range(60)  #green
        self.right_border_color = (0, 0, 255) # utilize same HUE value
        self.left_border_color = (0, 255, 0)
        self.contour_min_size = 50
        self.dwn_smpl = dwn_smpl
        self.cam_img_pub = rospy.Publisher("hmi_tracker_image", Image, queue_size=1)
        self.__cv_bridge = CvBridge()
        self.debug = debug

    def get_hue_color_range(self, color_base):
        sensitivity = 15
        upper = (color_base + sensitivity, 255, 255)
        lower = (color_base - sensitivity, 60, 60)
        return (lower, upper)

    def __fill_contours(self, img, contour_pts):
        cv2.drawContours(img, [contour_pts], 0, color=255, thickness=-1)
        self.__remove_contour(img, contour_pts)

    def __remove_contour(self, img, contour_pts):
        # contour has invalid pixels, which do not belong to the hand
        for x, y in np.squeeze(contour_pts):
            img[y][x] = 0

    def is_center_within_contour(self, center, contour):
        return cv2.pointPolygonTest(contour, (center[0], center[1]), True) >= 0

    def publish_img_if_required(self, img):
        # the topic is not published unless anyone is subcribed
        if (self.cam_img_pub.get_num_connections() > 0):
            img_msg = self.__cv_bridge.cv2_to_imgmsg(img)
            self.cam_img_pub.publish(img_msg)

    def preprocess_img(self, img_msg):
        img = self.__cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        final_size = (img.shape[1] / self.dwn_smpl, img.shape[0] / self.dwn_smpl)
        return cv2.resize(img, final_size)

    def find_hands(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        right_hand = self.__find_hand(hsv, self.right_color_range)
        left_hand = self.__find_hand(hsv, self.left_color_range)

        if right_hand is not None:
            self.__display_hand_on_img(img, right_hand, self.right_border_color)
        if left_hand is not None:
            self.__display_hand_on_img(img, left_hand, self.left_border_color)

        if (self.debug):
            if left_hand is not None:
                img = self.stack_mask(img, left_hand[3], "Left mask", self.left_border_color)
            if right_hand is not None:
                img = self.stack_mask(img, right_hand[3], "Right mask", self.right_border_color)
        self.publish_img_if_required(img)
        return left_hand, right_hand

    def stack_mask(self, img, mask, text, color):
        pos_x = 10 + img.shape[1]
        stacked_img = np.hstack((img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        self.__overlay_text(img, text, stacked_img, pos_x, color)
        return stacked_img

    def __overlay_text(self, img, text, stacked_img, pos_x, color):
        cv2.putText(stacked_img, text, (pos_x, img.shape[0] - 10), cv2.FONT_ITALIC, 2.0 / self.dwn_smpl, thickness = 8 / self.dwn_smpl, color = color)

    def __display_hand_on_img(self, img, hand_data, color):
        self.__overlay_circle_on_img(img, hand_data[0], color)
        self.__overlay_box_on_img(hand_data[1], img, color)

    def __overlay_circle_on_img(self, image, pos, color):
        cv2.circle(image,
               center=(int(pos[0]), int(pos[1])),
               radius=2,
               color=color,
               thickness=2,
               lineType=8,
               shift=0)
  
    def __overlay_box_on_img(self, bound_box, img, color):
        box = cv2.boxPoints(bound_box)
        box = np.int0(box) 
        cv2.drawContours(img, [box], 0, color, 2)
    
    def __find_hand(self, hsv_img, color_range):
        result = None
        mask = cv2.inRange(hsv_img, color_range[0], color_range[1])
        _, contours, _ = cv2.findContours(mask.astype("uint8"), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours:
            contour_pts = max(contours, key=lambda cont: cont.size)
            if contour_pts.size > self.contour_min_size / self.dwn_smpl:
                rect = cv2.minAreaRect(contour_pts)
                center = np.array(rect[0]).astype("int")
                if not self.is_center_within_contour(center, contour_pts):
                    # the closest point INSIDE contour, otherwise the center's height can be incorreclty calculated
                    center = np.squeeze(min(contour_pts, key=lambda p: distance.euclidean(p, center)))
                single_hand_img = np.zeros(mask.shape)
                self.__fill_contours(single_hand_img, contour_pts)
                result = (center, rect, contour_pts, mask, single_hand_img)
        return result
