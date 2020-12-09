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
        self.__marker_scale = Vector3(self.__k* 0.05, self.__k * 0.1, 0.05) # z - for text size
        self.__marker_color = ColorRGBA(*color)
        self.__text_color = ColorRGBA(0,0,1,1)
        self.__frame = parent_frame_id
        
    def __get_calibration_marker(self, calib):
        m = Marker(type = Marker.TEXT_VIEW_FACING, 
                        pose = Pose(orientation = i_quat), 
                        action = Marker.ADD, 
                        scale = self.__marker_scale, 
                        color = self.__text_color, 
                        text = calib.short_format(),
                        ns = "calibration", id = 1) 
        m.header.frame_id = self.__frame
        return m

    def publish_data_if_required(self, v, calib):
        if (self.pub.get_num_connections() > 0):
            self.__publish_speed_markers(v, calib)

    def __get_arrow(self, end_point, ns, id):
        m = Marker(type = Marker.ARROW, 
                    pose = Pose(orientation = i_quat), 
                    action = Marker.ADD, 
                    scale = self.__marker_scale, 
                    color = self.__marker_color, 
                    points = [Point(), end_point], 
                    ns = ns, id = id)
        m.header.frame_id = self.__frame
        return m

    def __publish_speed_markers(self, speed_vec, calib):
        ma = MarkerArray(markers = [
            self.__get_arrow(Point(x = self.__k * speed_vec.x), "x", 1),
            self.__get_arrow(Point(y = self.__k * speed_vec.y), "y", 2),
            self.__get_arrow(Point(z = self.__k * speed_vec.z), "z", 3)])

        if calib is not None:
            ma.markers.append(self.__get_calibration_marker(calib))
        self.pub.publish(ma)

