from visualization_msgs.msg import *
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import *
import rospy

import config

i_quat = Quaternion(0,0,0,1) # identity quaternion

class RVizVisualiser:
    def __init__(self, color, topic, parent_frame_id,  marker_scale):
        self.pub = rospy.Publisher(topic, MarkerArray, queue_size=1) 
        self.__k = marker_scale # scale modifier
        self.__max_vector_length = 50
        self.__marker_scale = Vector3(self.__k* 0.015, self.__k * 0.03, 0)
        self.__text_scale = Vector3(0, 0, 0.05)
        self.__marker_color = ColorRGBA(*color)
        self.__text_color = ColorRGBA(0,0,1,1)
        self.__frame = parent_frame_id

    def publish_data_if_required(self, v, calib):
        if (self.pub.get_num_connections() > 0):
            self.__publish_speed_markers(v, calib)

    def __get_arrow(self, end_point, ns, id):
        m = Marker(type = Marker.ARROW, 
                    pose = Pose(orientation = i_quat), 
                    action = Marker.ADD, 
                    scale = self.__marker_scale, 
                    color = self.__marker_color, 
                    points = [end_point, Point()], 
                    ns = ns, id = id)
        m.header.frame_id = self.__frame
        return m

    def __get_calibration_marker(self, calib):
        m = Marker(type = Marker.TEXT_VIEW_FACING, 
                        pose = Pose(orientation = i_quat), 
                        action = Marker.ADD, 
                        scale = self.__text_scale, 
                        color = self.__text_color, 
                        text = calib.short_format(),
                        ns = "calibration", id = 1) 
        m.header.frame_id = self.__frame
        return m

    def __publish_speed_markers(self, speed_vec, calib):
        ma = MarkerArray(markers = [ 
            self.__get_arrow(Point(x = self.__get_component(speed_vec.x)), "x", 1),
            self.__get_arrow(Point(y = self.__get_component(speed_vec.y)), "y", 2),
            self.__get_arrow(Point(z = self.__get_component(speed_vec.z)), "z", 3)])

        if calib is not None:
            ma.markers.append(self.__get_calibration_marker(calib))
        self.pub.publish(ma)
    
    def __get_component(self, value):
        # 255 - max speed
        return self.__k * float(value) / 255.0 * self.__max_vector_length

