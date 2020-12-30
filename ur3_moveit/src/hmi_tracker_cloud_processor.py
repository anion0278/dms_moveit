import rospy
import numpy as np
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image

# it is possible to utilize only Depth msg, because it contains RGB data
pc_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('rgb', 12, PointField.UINT32, 1)]

class HmiTrackerCloudProcessor:
    def __init__(self, dwn_smpl):
        self.dwn_smpl = dwn_smpl

    def publish_emtpy_pc(self, pub, header):
        cloud_empty = pc2.create_cloud(header, pc_fields, [])
        pub.publish(cloud_empty)
    
    def get_img_coords(self, depth_img, point, override_height = None): 
        x = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["x"]
        y = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["y"]
        z = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["z"] # this is not acurate
        if override_height is not None:
            z = override_height
        return (x, y, z)

    def get_center_and_publish(self,pc_pub,hand_data, depth_img, header):
         # the topic is not published unless anyone is subcribed
        publish_pointcloud = pc_pub.get_num_connections() > 0

        blob_pts = np.where(hand_data[3] > 0)

        heights = []
        pc_hand = []
        for x, y in zip(blob_pts[0], blob_pts[1]):
            depth_img_point = depth_img[x * self.dwn_smpl, y * self.dwn_smpl]
            heights.append(depth_img_point["z"])
            if publish_pointcloud:
                try:
                    pc_hand.append(depth_img_point)
                except:
                    pass # when point is on the edge of image

        if publish_pointcloud:
            cloud_modified = pc2.create_cloud(header, pc_fields, pc_hand)
            pc_pub.publish(cloud_modified)

        cloud_center = self.get_img_coords(depth_img, hand_data[0], np.median(heights))
        return cloud_center