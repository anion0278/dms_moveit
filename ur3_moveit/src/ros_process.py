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
import roslaunch
import rosgraph
import rosservice
import moveit_commander


def run_process_async(command, shell=True):
    return subprocess.Popen(command, shell=shell)


def run_process_sync(command):
    os.system(command)


def kill_process_by_name(process_name):
    for proc in psutil.process_iter():
        if proc.name() == process_name:
            proc.kill()


def start_roscore():
    process = run_process_async("roscore")
    rospy.wait_for_service("/rosout/get_loggers")


def is_roscore_running():
    return rosgraph.is_master_online()


def close_roscore():
    kill_process_by_name("roscore")
    kill_process_by_name("rosmaster")
    kill_process_by_name("roslaunch")
    kill_process_by_name("rosout")
    kill_process_by_name("move_group")
    kill_process_by_name("robot_state_pub")
    print("roscore killed !")


def wait_for_services_or_timeout(services, timeout, timeout_callback=None):
    time_spent = 0
    while timeout >= time_spent and all(not is_rosservice_running(x) for x in services):
        time.sleep(1)
        time_spent += 1
    if (time_spent >= gazebo_timeout):
        print("Time is up!")
        timeout_callback()
        raise RuntimeError("Timeout!")


def is_rosservice_running(service_name):
    services = rosservice.get_service_list()
    return service_name in services


world_props_service = "/gazebo/get_world_properties"
gazebo_timeout = 15


class GazeboSim:
    def __init__(self, is_obstacle_present, is_workspace_limited):
        self.gazebo_process = None
        self.is_obstacle_present = is_obstacle_present
        self.is_workspace_limited = is_workspace_limited

    def start(self, additional_params=""):
        self.__close_gazebo()
        self.gazebo_process = self.__start_gazebo_process(additional_params)

    def __start_gazebo_process(self, additional_params):
        package = 'ur3_moveit'
        node_name = 'start_sim.launch'

        # very concrete, needs refactoring
        command = "roslaunch {0} {1} spawn_obstacle:={2} {3}".format(
            package, node_name, str(self.is_obstacle_present).lower(), additional_params)

        if (self.is_workspace_limited):
            command += " xacro_file_name:=ur3_workspace_limitation.xacro"

        process = run_process_async(command)
        self.wait_for_gazebo_or_timeout()
        return process

    def wait_for_gazebo_or_timeout(self):
        wait_for_services_or_timeout(
            services=[
                "/arm_controller/query_state",
                "/move_group/plan_execution/set_parameters",
                world_props_service],
                timeout=gazebo_timeout,
                timeout_callback= self.end)

        self.__world_properties_service = rospy.ServiceProxy(
            world_props_service, GetWorldProperties)
        self.__model_state_service = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)
        self.__reset_sim_service = rospy.ServiceProxy(
            "/gazebo/reset_simulation", Empty)
        self.__reset_world_service = rospy.ServiceProxy(
            "/gazebo/reset_world", Empty)
        self.__pause_physics_service = rospy.ServiceProxy(
            "/gazebo/pause_physics", Empty)
        self.__unpause_physics_service = rospy.ServiceProxy(
            "/gazebo/unpause_physics", Empty)

    def get_sim_objects_names_list(self):
        response = self.__world_properties_service()
        return response.model_names

    def get_model_pos(self, model_name):
        return self.__model_state_service(model_name=model_name)

    def reset_sim(self):
        print("Reseting simulation...")
        self.__pause_physics_service()
        time.sleep(0.1)
        # self.__reset_sim_service()
        self.__reset_world_service()
        time.sleep(0.1)
        self.__unpause_physics_service()
        time.sleep(0.2)

    def __close_gazebo(self):
        kill_process_by_name("gzserver")
        kill_process_by_name("gzclient")
        if (self.gazebo_process != None):
            os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)

    def end(self):
        self.__close_gazebo()
        close_roscore()

