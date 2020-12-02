from visualization_msgs.msg import *
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import *
import rospy

import config

i_quat = Quaternion(0,0,0,1) # identity quaternion

class RVizVisualiser:
    def __init__(self, color, topic, parent_frame_id,  marker_scale):
        self.pub = rospy.Publisher(topic, MarkerArray, queue_size=1) 
        self.__k = marker_scale # marker scale
        self.__marker_scale = Vector3(self.__k* 0.05, self.__k * 0.1, 0)
        self.__marker_color = ColorRGBA(*color)
        self.__frame = parent_frame_id

    def publish_if_required(self, v):
        if (self.pub.get_num_connections() > 0):
            self.__publish_speed_markers(v)

    def __get_arrow(self, points, ns, id):
        m = Marker(type = Marker.ARROW, 
                    pose = Pose(orientation = i_quat), 
                    action = Marker.ADD,
                    scale = self.__marker_scale, 
                    color = self.__marker_color, 
                    points = points , 
                    ns = ns, id = id)
        m.header.frame_id = self.__frame
        return m

    def __publish_speed_markers(self, speed_vec): # TODO possible more refact.
        points1 = [Point(), Point(x = self.__k * speed_vec.x)]
        m1 = self.__get_arrow(points1, "x", 1)

        points2 = [Point(), Point(y = self.__k * speed_vec.y)]
        m2 = self.__get_arrow(points2, "y", 2)

        points3 = [Point(), Point(z = self.__k * speed_vec.z)]
        m3 = self.__get_arrow(points3, "z", 3)

        ma = MarkerArray(markers = [m1, m2, m3])
        self.pub.publish(ma)

