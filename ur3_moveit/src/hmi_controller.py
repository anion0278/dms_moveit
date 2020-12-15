#!/usr/bin/env python

import rospy
import numpy as np
import sys
import ros_numpy

import config
import hmi_controller_notificator as nt
import hmi_controller_processor as pr
import hmi_controller_calibration_manager as cb
import hmi_visualisation as vis
import hmi_controller_ble as ble
import hmi_disconnector
import util_common as util


debug = False  

class HmiController():
    def __init__(self, device_name, use_world_frame = False):
        self.node_name = device_name
        self.device_name = device_name

        self.__ble = ble.BleManager(self.device_name, debug, self.__main_loop, self.__process_hmi_data)
        self.notificator = nt.VibroNotificator(self.device_name)
        self.calibrator = cb.CalibrationManager(self, self.node_name, use_world_frame)
        self.visualizer = vis.RVizVisualiser(config.get_rviz_color(self.device_name), self.node_name + "_markers", self.calibrator.calibr_frame_id, 1)
        self.processor = pr.DataProcessor(self.device_name, self, self.calibrator, self.visualizer, self.notificator)

    def run(self):
        rospy.init_node(self.node_name)
        self.__ble.start()

    def __process_hmi_data(self, data):
        if debug:
            print(data)
        try:
            self.processor.process_hmi_data(data)
        except Exception as e:
            print("HMI data processing error: %s " % e.message)

    def __main_loop(self):
        self.notificator.init_params()
        self.__notify_ready()
        while True:
            self.set_hmi_notification()

    def set_hmi_notification(self):
        __time_step_s = 0.05
        command = self.notificator.calc_intensity()
        
        if isinstance(command, nt.TimedCommand):
            while command.time > 0:
                self.send_speed_command(command.intensity)
                rospy.sleep(__time_step_s)
                command.time = command.time - __time_step_s
                new_command = self.notificator.calc_intensity()
                if (isinstance(new_command, nt.TimedCommand) and new_command.intensity > command.intensity)  or new_command > command.intensity:
                    if isinstance(new_command, nt.TimedCommand):
                        self.send_speed_command(new_command.intensity)
                    else:
                        self.send_speed_command(new_command)
                    break

        else:
            self.send_speed_command(command) 
            rospy.sleep(rospy.Duration(secs=0, nsecs=5000))

    def send_speed_command(self, speed):
        # !!!!!!!!!!!!!!!!!!!!!!!!! debug speeed publish
        if self.processor.current_orientation is not None:
            ros_vec = self.processor.get_speed_vector(speed)
            speed_vec = speed * ros_numpy.numpify(ros_vec)
            motor_speeds = self.notificator.get_motor_speeds(speed_vec)
            self.send_speed(motor_speeds)

    def __notify_ready(self):
        offsets = self.calibrator.restore_imu_offsets()
        if offsets is None:
            self.send_handshake()
        else:
            self.send_offsets(offsets)
        self.calibrator.start_calibration_service()
        print("Device %s is ready." % self.device_name)

    def request_offsets(self):
        self.__ble.send_text("request-offsets")

    def send_speed(self, speed_comps):
        msg = self.__format_speed_msg(speed_comps)
        self.__ble.send_text(msg)

    def send_offsets(self, offsets):
        self.__ble.send_text("offsets:" + offsets)

    def send_handshake(self):
        self.__ble.send_text("handshake")

    def __format_speed_msg(self, speed_comps):
        speed_comps_strs = []
        for val in speed_comps:
            speed_comps_strs.append(str(int(val)).zfill(3))
        return "X{0}Y{1}Z{2}-X{3}-Y{4}-Z{5}".format(*speed_comps_strs) 

if __name__ == "__main__":

    use_world = False
    if not "node" in sys.argv:
        print("STANDALONE MODE !")
        use_world = True
        hmi_disconnector.restart_adapter()
        sys.argv.append(config.hmi_right)
        # sys.argv.append(config.hmi_left)

    if debug: util.print_all_args()
        
    hmi = HmiController(sys.argv[1], use_world)
    hmi.run()

    