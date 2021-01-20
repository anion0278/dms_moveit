import rospy
import numpy as np
import ros_numpy as rp
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointField
from scipy.spatial import distance
from icecream import ic # perfect for debuging purposes

import open3d # pip install pyrsistent==0.16.1 open3d-python 
# because simple installation will fail for Python 2.7 !
from open3d_ros_helper import open3d_ros_helper as orh
from open3d.open3d.geometry import statistical_outlier_removal

import util_common as utils

pc_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1)]
    # ,PointField('rgb', 12, PointField.UINT32, 1)] # to publish color

class HmiTrackerCloudProcessor:
    def __init__(self, dwn_smpl):
        self.dwn_smpl = dwn_smpl
        self.max_radius_m = 0.11

    def publish_emtpy_pc(self, pub, header):
        # publish always
        cloud_empty = pc2.create_cloud(header, pc_fields, [])
        pub.publish(cloud_empty)
    
    def get_img_coords(self, depth_img, point, copy_rgb = False): 
        x = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["x"]
        y = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["y"]
        z = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["z"] 
        if copy_rgb:# unused for now
            rgb = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["rgb"] 
        return (x, y, z)

    def get_center_and_publish(self,pc_pub, hand_data, depth_img, header):
        # the topic is not published unless anyone is subcribed
        publish_pointcloud = pc_pub.get_num_connections() > 0

        blob_pts = self.__get_valid_points(hand_data.single_hand_mask, depth_img)
        if len(blob_pts) == 0: return None, None

        cloud_center = self.__get_valid_cloud_center(depth_img, hand_data.center, blob_pts)
            
        r_max = 0
        pc_hand = [] # pointCloud points
        for x, y in blob_pts:
            depth_img_point = self.get_img_coords(depth_img, (x, y))
            pc_hand.append(depth_img_point)
 
        # TODO optimize
        cloud_modified = pc2.create_cloud(header, pc_fields, pc_hand)
        o3dpc = orh.rospc_to_o3dpc(cloud_modified)
        filtered_cloud, outliers_indices = statistical_outlier_removal(o3dpc, nb_neighbors= len(o3dpc.points) / self.dwn_smpl, std_ratio=0.00001)
        ros_cl = orh.o3dpc_to_rospc(filtered_cloud)
        ros_cl.header = cloud_modified.header

        if publish_pointcloud:  
            pc_pub.publish(ros_cl)

        for point in filtered_cloud.points:
            r = distance.euclidean(point, cloud_center)
            if r > r_max:
                r_max = r

        if r_max > self.max_radius_m: 
            r_max = self.max_radius_m

        return cloud_center, r_max 

    def __get_valid_cloud_center(self, depth_img, img_center, blob_pts):
        cloud_center = self.get_img_coords(depth_img, img_center) # first try the actual center
        if utils.has_nan_values(cloud_center):
            not_nan_center_img = min(blob_pts, key=lambda p: distance.euclidean(p, img_center))
            cloud_center = self.get_img_coords(depth_img, not_nan_center_img)
        return cloud_center

    def __get_valid_points(self, single_hand_mask, depth_img): # valid hmi points on 2D img
        blob_pts_all = np.where(single_hand_mask > 0)
        blob_pts = [] # array of X, Y where each pixel is WHITE and is not NAN
        for y, x in zip(blob_pts_all[0], blob_pts_all[1]): # x , y must be exchanged (open CV stuff)
            if not utils.has_nan_values(self.get_img_coords(depth_img, (x,y))):
                blob_pts.append((x,y))
        return blob_pts