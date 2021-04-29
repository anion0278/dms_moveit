import re
import numpy as np
import ros_numpy
import rospy
import tf2_ros 
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import *
from tf.transformations import *
import tf2_geometry_msgs as tf2_tr

import util_ros_msgs as util_ros
import hmi_controller_calibration_manager as cal
import config

regex_calibration_pattern = "(\d)(\d)(\d)(\d)"
regex_imu_pattern = "Q(-?\d)(\d{2})(-?\d)(\d{2})(-?\d)(\d{2})(-?\d)(\d{2})" + regex_calibration_pattern
regex_offsets_pattern = regex_calibration_pattern + "-(.{22})"

class DataProcessor():
    def __init__(self, controller, calibrator, visualizer, notifier):
        self.calibrator = calibrator
        self.notifier = notifier
        self.visualizer = visualizer
        self.controller = controller
        
        self.__collision_vec_sub = rospy.Subscriber(config.collision_vec_topic, MarkerArray, self.process_collsion_vector, queue_size=5)
        self.currect_collision_vec = [0,0,0]
        self.current_orientation = None
        self.current_imu_status = None

    def get_speed_vector(self):
        vs = Vector3Stamped(vector = Vector3(*self.currect_collision_vec))
        qa = quaternion_inverse(self.current_orientation)
        trs = TransformStamped(transform = Transform(rotation = Quaternion(*qa)))
        ros_vec = tf2_tr.do_transform_vector3(vs, trs).vector
        return ros_numpy.numpify(ros_vec) # normalized vector

    def process_hmi_data(self, data):
        match = re.search(regex_imu_pattern, data, re.IGNORECASE)
        if match: 
            self.__process_imu_data(np.array(match.groups()))
            return

        # it is really important to process string as bytearray
        match = re.search(regex_offsets_pattern, bytearray(data), re.IGNORECASE)
        if match:
            self.__process_offsets(np.array(match.groups()))
            return

    def process_collsion_vector(self, marker_array_msg):
        data_namespace = self.controller.device_name+"_vector"
        if any(data_namespace == m.ns for m in marker_array_msg.markers):
            points = (m.points for m in marker_array_msg.markers if data_namespace == m.ns).next()
            v = ros_numpy.numpify(points[0]) - ros_numpy.numpify(points[1])
            vec_len = np.linalg.norm(v)
            if vec_len != 0: 
                self.currect_collision_vec = v / vec_len # vector should be normalized !
                # print("New vector (%s): %s" % (self.controller.device_name, self.currect_collision_vec))
                # print("New vector length [m]: %s" % vec_len)

    def __get_compressed_quarternion(self, data): 
        floats = np.zeros(4)
        for i in range(len(floats)):
            floats[i] = float(data[i * 2]+'.'+data[i * 2 + 1])
        q = Quaternion(*floats)
        return q

    def __process_offsets(self, data):
        status = cal.ImuStatus(data[0], data[1], data[2], data[3])
        if status.is_fully_calibrated():
            offsets = data[4]
            print("Got valid offsets")
            self.calibrator.store_imu_offsets(offsets)
        else:
            print("Offsets cannot be stored! IMU is not fully calibrated: %s " % status)

    def __process_imu_data(self, data):
            # print(self.node_name +"->"+data)
            q_real = ros_numpy.numpify(self.__get_compressed_quarternion(data))
            if not np.array_equal(q_real, np.zeros(4)): # this check is required
                self.current_orientation = self.calibrator.get_calibrated_quat(q_real)
                self.current_imu_status = cal.ImuStatus(data[8], data[9], data[10], data[11])

                self.calibrator.publish_tf(self.current_orientation)