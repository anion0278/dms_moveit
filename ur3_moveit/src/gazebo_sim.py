#!/usr/bin/python

import time
import rospy
import os
import subprocess
import roslaunch
import rospkg
import signal
import psutil
import yaml
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from std_srvs.srv import Empty
from StringIO import StringIO

class GazeboSim():
    def __init__(self, is_obstacle_present):
        self.gazebo_process = None
        self.is_obstacle_present = is_obstacle_present

    def start_gazebo(self):
        self.close_gazebo()
        self.gazebo_process = self.start_sub_process()

    def start_sub_process(self):
        package = 'ur3_moveit'
        node_name = 'start_sim.launch'

        # very concrete, needs refactoring
        command = "roslaunch {0} {1} spawn_obstacle:={2}".format(
            package, node_name, str(self.is_obstacle_present).lower())

        process = subprocess.Popen(command, shell=True)

        rospy.wait_for_service("/arm_controller/query_state")
        rospy.wait_for_service("/move_group/plan_execution/set_parameters")
        world_props_service = "/gazebo/get_world_properties"
        rospy.wait_for_service(world_props_service)
        self.__world_properties_service = rospy.ServiceProxy(
            world_props_service, GetWorldProperties)
        self.__model_state_service = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)
        self.__reset_sim_service = rospy.ServiceProxy(
            "/gazebo/reset_world", Empty)
        time.sleep(1)

        state = process.poll()
        if state is None:
            print("process is running fine")
            return process
        elif state < 0:
            print("Process terminated with error")
        elif state > 0:
            print("Process terminated without error")

    def get_sim_objects_names_list(self):
        response = self.__world_properties_service()
        return response.model_names

    def get_model_pos(self, model_name):
        return self.__model_state_service(model_name=model_name)

    def reset_sim(self):
        print("Reseting simulation...")
        self.__reset_sim_service()
        time.sleep(0.5)

    def close_gazebo(self):
        self.__kill_process_by_name("gzserver")
        self.__kill_process_by_name("gzclient")
        if (self.gazebo_process != None):
            os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)

    def __kill_process_by_name(self, process_name):
        for proc in psutil.process_iter():
            if proc.name() == process_name:
                proc.kill()