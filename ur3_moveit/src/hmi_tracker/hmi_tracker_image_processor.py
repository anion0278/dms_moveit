import cv2
from scipy.spatial import distance
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys, os
sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )
from config import config

class HmiImageData:
    def __init__(self, center, bounding_box, blob_pts, full_color_mask):
        self.center = center
        self.bounding_box = bounding_box
        self.blob_pts = blob_pts
        self.full_color_mask = full_color_mask

class ColorRange:
    def __init__(self, lower, upper):
        self.upper = upper
        self.lower = lower
        self.h_lim = 179
    
    # HSV value ranges = 0-180, 0-255, 0-255
    def is_split(self): # does not handle range overflow
        return self.lower[0] < 0 or self.upper[0] > self.h_lim

    def get_split_ranges(self): 
        return [ColorRange((0, self.lower[1], self.lower[2]), self.upper), 
            ColorRange((self.h_lim+self.lower[0], self.lower[1], self.lower[2]), (self.h_lim, self.upper[1], self.upper[2]))]

class HmiTrackerImageProcessor:
    def __init__(self, dwn_smpl, debug):
        self.right_border_color = config.color_right[::-1] * 255 
        self.left_border_color = config.color_left[::-1] * 255 # rgb to bgr
        self.right_color_range = self.get_hsv_red_range()  
        self.left_color_range = self.get_hsv_green_range() 
        
        self.contour_min_size = 150.0 # down sampling is applied in code!
        self.contour_max_size = 1000.0
        self.dwn_smpl = dwn_smpl
        self.cam_img_pub = rospy.Publisher("hmi_tracker_image", Image, queue_size=1)
        self.__cv_bridge = CvBridge()
        self.debug = debug

    def get_hsv_green_range(self):
        lower = (72, 100, 100)
        upper = (94, 255, 250)
        return ColorRange(lower, upper)

    def get_hsv_red_range(self):
        lower = (-30, 80, 90) # 150 - 179
        upper = (10, 255, 255) 
        return ColorRange(lower, upper)

    def get_hsv_color_range(self, color_base):
        sensitivity = 10
        upper = (color_base + sensitivity, 255, 220)
        lower = (color_base - sensitivity, 100, 100)
        return ColorRange(lower, upper)

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
                # http://hanzratech.in/2015/02/07/caveat-thresholding-hue-component.html#:~:text=The%20HSV%20values%20for%20true,we%20will%20use%20the%20cv2.
                img = self.stack_mask(img, left_hand.full_color_mask, "Left mask", self.left_border_color)
            if right_hand is not None:
                img = self.stack_mask(img, right_hand.full_color_mask, "Right mask", self.right_border_color)
        return left_hand, right_hand, img

    def stack_mask(self, img, mask, text, color):
        pos_x = 10 + img.shape[1]
        stacked_img = np.hstack((img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        self.__overlay_text(img, text, stacked_img, pos_x, color)
        return stacked_img

    def __overlay_text(self, img, text, stacked_img, pos_x, color):
        # scale & thickness should be calculated relative to the size of the image !
        cv2.putText(stacked_img, text, (pos_x, img.shape[0] - 10), cv2.FONT_ITALIC, 1.0 / self.dwn_smpl, thickness = 2 / self.dwn_smpl, color = color)

    def __display_hand_on_img(self, img, hand_data, color):
        self.__overlay_circle_on_img(img, hand_data.center, color)
        self.__overlay_box_on_img(hand_data.bounding_box, img, color)

    def __overlay_circle_on_img(self, image, pos, color):
        cv2.circle(image,
               center=(int(pos[0]), int(pos[1])),
               radius=2,
               color=color,
               thickness=2,
               lineType=8,
               shift=0)
  
    def __overlay_box_on_img(self, bounding_box, img, color):
        box = cv2.boxPoints(bounding_box)
        cv2.drawContours(img, [np.int0(box)], 0, color, 2)
    
    def __find_hand(self, hsv_img, color_range):
        color_mask = self.get_color_mask(hsv_img, color_range)

        # joins the regions - shows better results
        color_mask = cv2.dilate(color_mask, None, iterations=8 / self.dwn_smpl)
        color_mask = cv2.erode(color_mask, None, iterations=6 / self.dwn_smpl)

        _, contours, _ = cv2.findContours(color_mask.astype("uint8"), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours:
            contour_pts = max(contours, key=lambda cont: cont.size)
            if contour_pts.size > self.contour_min_size / self.dwn_smpl and \
                contour_pts.size < self.contour_max_size / self.dwn_smpl:
                rect = cv2.minAreaRect(contour_pts)
                center = np.array(rect[0]).astype("int")

                single_hand_mask = np.zeros(color_mask.shape)
                self.__fill_contours(single_hand_mask, contour_pts)

                blob_pts = np.where(single_hand_mask > 0)
                if len(blob_pts[1]) == 0:
                    return None # can occur due to removal of the contour
                # after contour is removed, blob can be empty, leading to copying of all points from PC MSG

                return HmiImageData(center, rect, blob_pts, color_mask)
        return None

    def get_color_mask(self, hsv_img, color_range):
        mask = None
        if color_range.is_split():# correctly handles RED color in HSV
            comp_ranges = color_range.get_split_ranges()
            mask = self.get_color_mask(hsv_img, comp_ranges[0]) + self.get_color_mask(hsv_img, comp_ranges[1])
        else:
            mask = cv2.inRange(hsv_img, color_range.lower, color_range.upper)
        return mask
