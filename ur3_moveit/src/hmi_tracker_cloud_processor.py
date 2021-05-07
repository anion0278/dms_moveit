import numpy as np
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
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

class HmiTrackerCloudProcessor:
    def __init__(self, dwn_smpl):
        self.dwn_smpl = dwn_smpl
        self.max_radius_m = 0.09

    def publish_emtpy_pc(self, pub, frame_id):
        cloud_empty = pc2.create_cloud(Header(frame_id = frame_id), pc_fields, [])
        pub.publish(cloud_empty)

    def publish_if_required(self, pc_pub, ros_pc):
        # the topic is not published unless anyone is subcribed
        if pc_pub.get_num_connections() > 0:  
            pc_pub.publish(ros_pc)

    def get_hmi_point_cloud(self, hand_data, pc_msg):
        blob_pts = hand_data.blob_pts
        if len(blob_pts[1]) == 0:
            raise ValueError("Hand blob did not contain any points, this would lead to full copy of point cloud!")
        pc_gen = pc2.read_points(pc_msg, field_names = ["x","y","z"], uvs = zip(blob_pts[1]*self.dwn_smpl, blob_pts[0]*self.dwn_smpl), skip_nans=True)
        pc_hand = np.array([x for x in pc_gen])
        return pc_hand

    def filter_point_cloud(self, pc_hand):
        o3dpc = open3d.geometry.PointCloud()
        o3dpc.points = open3d.utility.Vector3dVector(pc_hand[:, :3])
        neighbors = int(len(o3dpc.points) / self.dwn_smpl)
        filtered_cloud, _ = statistical_outlier_removal(o3dpc, nb_neighbors=neighbors, std_ratio=0.00001)
        return filtered_cloud

    def get_pc_bounding_sphere(self, filtered_cloud_o3d, frame_id):
        centroid = np.median(filtered_cloud_o3d.points, axis=0)
        if utils.has_nan_values(centroid):
            return None

        ros_cl = orh.o3dpc_to_rospc(filtered_cloud_o3d)
        ros_cl.header.frame_id = frame_id

        r_max = 0
        for point in filtered_cloud_o3d.points:
            r = distance.euclidean(point, centroid)
            if r > r_max:
                r_max = r

        if r_max > self.max_radius_m: 
            r_max = self.max_radius_m

        return centroid, r_max, ros_cl


