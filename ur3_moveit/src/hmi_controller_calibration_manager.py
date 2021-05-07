#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import TriggerRequest, Trigger, TriggerResponse
import pickle as serializer # replace by json if more readable data format is needed
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_ros

import config
import util_ros_msgs as util_ros

class CalibrationManager(): # TODO separate into Calibration and Transform managers
    def __init__(self, controller, node_name, use_world_frame, debug):
        self.calibr_service = None
        self.debug = debug

        self.controller = controller
        self.calibr_service_name = node_name + config.calibr_service
        self.offsets_file = config.get_offsets_file_path(node_name)
        self.frame_calib_file = config.get_frame_calibration_file_path(node_name)

        self.tf_pub = tf2_ros.StaticTransformBroadcaster()
        self.tf_buf = tf2_ros.Buffer(cache_time = config.tf_viz_decay_duration_s) 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.__tracked_frame_id = controller.device_name
        if use_world_frame:
            self.__tracked_frame_id = "world"
        self.calibr_frame_id = controller.device_name+"_calibrated"
        self.zero_frame_id = controller.device_name+"_north"
        self.frame_calibration = self.restore_frame_calibration()

    def is_hmi_recognized(self):
        return self.tf_buf.can_transform("world", self.__tracked_frame_id, rospy.Time(0)) 

    def get_calibrated_quat(self, quaternion_real):
        return quaternion_multiply(self.frame_calibration, quaternion_real)

    def publish_tf(self, quaternion_hmi):
        if self.debug: 
            tfs_north_direction = util_ros.get_stamped_transform(self.__tracked_frame_id, self.zero_frame_id, util_ros.get_norm_quaternion_msg(self.frame_calibration))
            self.tf_pub.sendTransform(tfs_north_direction) # correct only when calibrated
        
        tfs_calibrated = util_ros.get_stamped_transform(self.__tracked_frame_id, self.calibr_frame_id, util_ros.get_norm_quaternion_msg(quaternion_hmi))
        self.tf_pub.sendTransform(tfs_calibrated)

    def start_calibration_service(self):
        self.calibr_service = rospy.Service(self.calibr_service_name, Trigger, self.calibration_procedure)

    def calibration_procedure(self, request):
        self.frame_calibration = self.__get_frame_calibration()
        self.controller.send_offsets_request_msg()
        self.__store_frame_calibration()
        return TriggerResponse(success=True,
            message= self.controller.device_name + " was succesfully calibrated.")

    def restore_imu_offsets(self):
        try:
            offsets = serializer.load(open(self.offsets_file, "rb"))
            if len(offsets) != 22:
                raise EnvironmentError("Error during deserializaion, check data!")
        except Exception as e:
            print("Could not restore IMU offsets! Ex: " + e.message)
            offsets = None
        return offsets

    def restore_frame_calibration(self): 
        try:
            quat = serializer.load(open(self.frame_calib_file, "rb"))
            if len(quat) != 4:
                raise EnvironmentError("Error during deserializaion, check data!")
        except Exception as e:
            print("Could not restore calibration! Using an indentity quaternion. Ex: " + e.message)
            quat = util_ros.get_identity_quat_arr()
        return quat

    def store_imu_offsets(self, offsets):
        serializer.dump(offsets, open(self.offsets_file, "w"))
        print("IMU offsets were stored in %s" % self.offsets_file)

    def __store_frame_calibration(self):
        serializer.dump(self.frame_calibration.tolist(), open(self.frame_calib_file, "w"))
        print("Frame calibration was stored in %s" % self.frame_calib_file)

    def __get_frame_calibration(self):
        # TODO here we need the "pure" orientation
        original_real_orientation = quaternion_multiply(
            quaternion_inverse(self.frame_calibration),
            self.controller.processor.current_orientation)
        return quaternion_inverse(original_real_orientation)


class ImuStatus(): 
    def __init__(self, sys, gyro, acc, mag):
        self.sys = int(sys)
        self.gyro = int(gyro)
        self.acc = int(acc)
        self.mag = int(mag)
    
    @property
    def is_ready_for_use(self):
        if self.gyro == 3 and self.mag == 3: # other params may change over time
            return True
        return False
    
    def is_fully_calibrated(self):
        if self.sys == 3 and self.gyro == 3 and self.acc == 3 and self.mag == 3:
            return True
        return False

    def short_format(self):
        return "S{0};G{1};A{2};M{3}".format(self.sys, self.gyro, self.acc, self.mag)

    def __str__(self):
        return "Sys: {0}; Gyro: {1}; Acc: {2}; Mag: {3}".format(self.sys, self.gyro, self.acc, self.mag)


if __name__ == "__main__":
    if "calibrate" in sys.argv:
        print("Requesting calibration...")
        
        service_ids = [config.hmi_left + config.calibr_service, config.hmi_right + config.calibr_service]
        services = []
        for ser in service_ids:
            try:
                rospy.wait_for_service(ser, timeout=0.1)
                services.append(rospy.ServiceProxy(ser, Trigger))
            except Exception as e: 
                print("Could not find %s service" % ser)

        for ser in services:
            result = ser(TriggerRequest())
            print(result.message)
        print("Finished")
    else:
        print("No parameters were provided !")



