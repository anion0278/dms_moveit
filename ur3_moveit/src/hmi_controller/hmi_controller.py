#!/usr/bin/env python

import rospy
import numpy as np
import ros_numpy
import sys
from icecream import ic
import setproctitle

import sys, os
sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )
from utils import util_common as util
from config import config
setproctitle.setproctitle(config.hmi_controller_process)

import controller_heartbeat_watchdog as watchdog
import hmi_controller_notifier as nt
import hmi_controller_processor as pr
import hmi_controller_calibration_manager as cb
import hmi_visualisation as vis
import hmi_controller_ble as ble
import hmi_controller_disconnector

debug = False  

class HmiController():
    def __init__(self, device_name, directed_vibration, use_world_frame = False):
        self.node_name = device_name
        self.device_name = device_name
        setproctitle.setproctitle(config.hmi_controller_process + ": %s" % device_name)

        rospy.init_node(self.node_name) # should be initialized before Calibration Manager (TFBuffer)
        self.__ble = ble.BleManager(self.device_name, debug, self.__main_loop, self.__process_hmi_data)
        self.notifier = nt.VibroNotifier(self.device_name, directed_vibration)
        self.calibrator = cb.CalibrationManager(self, self.node_name, use_world_frame, debug)
        self.visualizer = vis.RVizVisualiser(config.get_rviz_color(self.device_name), self.node_name + "_markers", self.calibrator.calibr_frame_id, 1)
        self.processor = pr.DataProcessor(self, self.calibrator, self.visualizer, self.notifier)
        self.watchdog_notifier = watchdog.HeartbeatSender(self.node_name)

    def run(self):
        self.__ble.start()

    def __process_hmi_data(self, data):
        self.watchdog_notifier.send_beat()
        if debug:
            print(data)
        try:
            self.processor.process_hmi_data(data)
        except Exception as e:
            print("HMI data processing error: %s " % e.message)

    def __main_loop(self):
        self.notifier.init_params()
        self.__notify_ready()
        while not rospy.is_shutdown():
            self.set_hmi_notification()
        print("Node %s: ROS shutdown" % self.device_name)
        sys.exit()

    def set_hmi_notification(self):
        notification = self.notifier.get_notification()
        if isinstance(notification, nt.ProlongedNotification):
            start_time = rospy.get_time()
            while notification.time > (rospy.get_time() - start_time):
                self.send_speed_command(notification) 

                new_notification = self.notifier.get_notification()
                if new_notification.intensity > notification.intensity:
                    if isinstance(new_notification, nt.ProlongedNotification):
                        self.send_speed_command(new_notification) 
                    else:
                        self.send_speed_command(new_notification) 
                    break 
            return

        if isinstance(notification, nt.PromptNotification):
            self.send_speed_command(notification)
            return
        
        raise AttributeError("Unrecognized type of notification")

    def send_speed_command(self, notification):
        util.set_param(config.notification_val_param+self.node_name, notification.intensity) # timeline data
        # ic(self.device_name, self.calibrator.is_hmi_recognized())
        if self.processor.current_orientation is not None and self.calibrator.is_hmi_recognized() :
            norm_vec = self.processor.get_speed_vector() # normalized vector
            self.visualizer.publish_data_if_required(norm_vec * notification.intensity, self.processor.current_imu_status)  # TODO visualization for SMAM
            motor_speeds = self.notifier.get_motor_speeds(notification, norm_vec)
            # ic("Motor speeds (%s): %s" % (self.device_name, motor_speeds))
            if not util.has_nan_values(motor_speeds): # FIXME refactoring
                self.send_speed_msg(motor_speeds)
            else: 
                # ic("Nan values!!")
                self.send_speed_msg([0,0,0, 0,0,0]) 
        else:
            # ic("Could not find TF for %s, HMI is not recognized" % self.device_name)
            self.send_speed_msg([0,0,0, 0,0,0]) # we need to sustain communication even if HMI is not recognized

    def __notify_ready(self):
        offsets = self.calibrator.restore_imu_offsets()
        if offsets is None:
            self.send_handshake_msg()
        else:
            self.send_offsets_msg(offsets)
        self.calibrator.start_calibration_service()
        print("Device %s is ready." % self.device_name)

    def send_offsets_request_msg(self):
        self.__ble.send_text("request-offsets")

    def send_speed_msg(self, speed_comps):
        msg = self.__format_speed_msg(speed_comps)
        self.__ble.send_text(msg)
        delay_time_s = 0.02
        rospy.sleep(delay_time_s)

    def send_offsets_msg(self, offsets):
        self.__ble.send_text("offsets:" + offsets)

    def send_handshake_msg(self):
        self.__ble.send_text("handshake")

    def __format_speed_msg(self, speed_comps):
        speed_comps_strs = []
        for val in speed_comps:
            speed_comps_strs.append(str(int(val)).zfill(3))
        return "X{0}Y{1}Z{2}-X{3}-Y{4}-Z{5}".format(*speed_comps_strs) 

if __name__ == "__main__":
    util.print_all_args()

    use_world = False
    if not "mode:=node" in sys.argv:
        print("STANDALONE MODE !")
        use_world = True
        hmi_controller_disconnector.restart_adapter()
        sys.argv.append(config.hmi_right)
        # sys.argv.append(config.hmi_left)
        debug = True

    device_name = sys.argv[1].replace("device_name:=","")
    
    directed_vibration = False
    if ("directed_vibration:=true" in sys.argv):
        print("(%s) Directed vibration enabled" % device_name)
        directed_vibration = True
    
    hmi = HmiController(device_name = device_name, directed_vibration = directed_vibration, use_world_frame = use_world)
    hmi.run()

    