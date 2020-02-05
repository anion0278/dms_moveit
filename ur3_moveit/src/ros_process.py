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


class Environment:
    def __init__(self, is_obstacle_present, is_workspace_limited, is_simulated):
        self.env_process = None
        self.is_obstacle_present = is_obstacle_present
        self.is_workspace_limited = is_workspace_limited
        self.is_simulated = is_simulated

    def start(self, additional_params=""):
        self.__close_env()
        self.env_process = self.__start_environment_process(additional_params)

    def __start_environment_process(self, additional_params):
        package = "ur3_moveit"

        if self.is_simulated:
            node_name = "start_sim"
        else:
            node_name = "start_real"

        command = "roslaunch {0} {1}.launch spawn_obstacle:={2} {3}".format(
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
                "/state_validity_service",
                "/get_planning_scene",
                "/clear_octomap",
                world_props_service],
            timeout=gazebo_timeout,
            timeout_callback=self.end)

        if (self.is_simulated):
            self.__world_properties_service = rospy.ServiceProxy(world_props_service, GetWorldProperties)
            self.__model_state_service = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            self.__reset_sim_service = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
            self.__reset_world_service = rospy.ServiceProxy("/gazebo/reset_world", Empty)
            self.__pause_physics_service = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            self.__unpause_physics_service = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

    def get_sim_objects_names_list(self):
        if self.is_simulated:
            response = self.__world_properties_service()
            return response.model_names
        else:
            return []

    def get_model_pos(self, model_name):
        return self.__model_state_service(model_name=model_name)

    def set_model_pos(self, model_name, pose):
        self.__model_state_service(model_name=model_name, pose = pose)

    def reset(self):
        if self.is_simulated:
            print("Reseting simulation...")
            self.__pause_physics_service()
            time.sleep(0.1)
            # self.__reset_sim_service()
            self.__reset_world_service()
            time.sleep(0.1)
            self.__unpause_physics_service()
            time.sleep(0.2)

    def pause_gazebo(self):
        self.__pause_physics_service()

    def unpause_gazebo(self): 
        self.__unpause_physics_service()

    def __close_env(self):
        kill_process_by_name("gzserver")
        kill_process_by_name("gzclient")
        if (self.env_process != None):
            os.killpg(os.getpgid(self.env_process.pid), signal.SIGTERM)

    def end(self):
        self.__close_env()
        close_roscore()
