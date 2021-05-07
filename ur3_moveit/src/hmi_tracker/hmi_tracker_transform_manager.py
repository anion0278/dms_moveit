
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped, Transform
import tf2_ros
from sensor_msgs.msg import PointCloud2
import PyKDL
from tf2_sensor_msgs.tf2_sensor_msgs import transform_to_kdl

class HmiTrackerTransformManager:
    def __init__(self):
        self.cam_trf = None
        self.tf_pub = tf2_ros.StaticTransformBroadcaster() 
        self.tf_buf = None

    def init_hmi_orientation(self, cam_pc_topic):
        self.tf_buf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buf) # required
        cam_frame_id = rospy.wait_for_message(cam_pc_topic, PointCloud2).header.frame_id
        self.cam_trf = self.tf_buf.lookup_transform(cam_frame_id, "world", rospy.Time(), timeout=rospy.Duration(0.1)).transform
        pass

    def publish_hmi_tf(self, stamped_pose, parent_frame_id, hmi_name):
        t = TransformStamped(transform = Transform(translation = stamped_pose.pose.position, rotation = self.cam_trf.rotation))
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame_id
        t.child_frame_id = hmi_name
        self.tf_pub.sendTransform(t)

    def get_hand_pose(self, frame_id, center_pos):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.pose.position.x = center_pos[0]
        p.pose.position.y = center_pos[1]
        p.pose.position.z = center_pos[2]
        p.pose.orientation = Quaternion(0, 0, 0, 1)
        return p

    def transform_pc(self, points, initial_frame_id, target_frame_id):
        trsf = self.tf_buf.lookup_transform(initial_frame_id, target_frame_id, rospy.Time(), timeout=rospy.Duration(0.1))
        t_kdl = transform_to_kdl(trsf)
        points_out = []
        for p_in in points:
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append((p_out[0], p_out[1], p_out[2]))
        return points_out
