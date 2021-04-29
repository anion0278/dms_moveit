import numpy as np
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
        self.max_radius_m = 0.09

    def publish_emtpy_pc(self, pub, header):
        cloud_empty = pc2.create_cloud(header, pc_fields, [])
        pub.publish(cloud_empty)
    
    def get_img_coords(self, depth_img, point, copy_rgb = False): 
        x = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["x"]
        y = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["y"]
        z = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["z"] 
        if copy_rgb:
            rgb = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["rgb"] 
        return (x, y, z)

    def get_center_and_publish(self,pc_pub, hand_data, depth_img, header):
        blob_pts_all = np.where(hand_data.single_hand_mask > 0)
        pc_gen = pc2.read_points(depth_img, field_names = ["x","y","z"], uvs = zip(blob_pts_all[1]*self.dwn_smpl, blob_pts_all[0]*self.dwn_smpl), skip_nans=True)
        pc_hand = np.array([x for x in pc_gen])

        if len(pc_hand) == 0: return None, None

        o3dpc = open3d.geometry.PointCloud()
        o3dpc.points = open3d.utility.Vector3dVector(pc_hand[:, :3])
        filtered_cloud, _ = statistical_outlier_removal(o3dpc, nb_neighbors= len(o3dpc.points) / self.dwn_smpl, std_ratio=0.00001)
        centroid = np.median(filtered_cloud.points, axis=0)
        ros_cl = orh.o3dpc_to_rospc(filtered_cloud)
        ros_cl.header = header

        # the topic is not published unless anyone is subcribed
        if pc_pub.get_num_connections() > 0:  
            pc_pub.publish(ros_cl)

        r_max = 0
        for point in filtered_cloud.points:
            r = distance.euclidean(point, centroid)
            if r > r_max:
                r_max = r

        if r_max > self.max_radius_m: 
            r_max = self.max_radius_m

        return centroid, r_max 
