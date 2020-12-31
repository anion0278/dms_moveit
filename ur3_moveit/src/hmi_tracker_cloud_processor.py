import rospy
import numpy as np
import ros_numpy as rp
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image
from scipy.spatial import distance


pc_fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1)]
    # ,PointField('rgb', 12, PointField.UINT32, 1)] # to publish color

class HmiTrackerCloudProcessor:
    def __init__(self, dwn_smpl):
        self.dwn_smpl = dwn_smpl
        self.max_radius_m = 0.12

    def publish_emtpy_pc(self, pub, header):
        cloud_empty = pc2.create_cloud(header, pc_fields, [])
        pub.publish(cloud_empty)
    
    def get_img_coords(self, depth_img, point, copy_rgb = False): 
        x = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["x"]
        y = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["y"]
        z = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["z"] 
        if copy_rgb:
            rgb = depth_img[point[1] * self.dwn_smpl, point[0] * self.dwn_smpl]["rgb"] 
            # unused for now
        return (x, y, z)

    def get_center_and_publish(self,pc_pub, hand_data, depth_img, header):
         # the topic is not published unless anyone is subcribed
        publish_pointcloud = pc_pub.get_num_connections() > 0

        blob_pts = np.where(hand_data.single_hand_mask > 0)

        cloud_center = self.get_img_coords(depth_img, hand_data.center)
        r_max = 0

        heights = []
        pc_hand = []
        for x, y in zip(blob_pts[0], blob_pts[1]):
            depth_img_point = self.get_img_coords(depth_img, (y * self.dwn_smpl, x * self.dwn_smpl))
            heights.append(depth_img_point)
            if publish_pointcloud:
                try:
                    pc_hand.append(depth_img_point)
                except:
                    pass # when point is on the edge of image

            r = distance.euclidean(depth_img_point, cloud_center)
            if r > r_max:
                r_max = r

        if publish_pointcloud:
            cloud_modified = pc2.create_cloud(header, pc_fields, pc_hand)
            pc_pub.publish(cloud_modified)

        if r_max > self.max_radius_m: 
            r_max = self.max_radius_m

        return cloud_center, r_max 