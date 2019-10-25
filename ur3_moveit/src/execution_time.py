#!/usr/bin/env python

import time
import robot_driver
import rospy
import os
import subprocess
import roslaunch
import rospkg
import signal
import psutil
import yaml

current_script_path = os.path.dirname(os.path.realpath(__file__))


class MeasurementLogger:
    def __init__(self):
        self.__log_file = open(os.path.join(
            current_script_path, "measurement.txt"), "a+")
        self.__separator = "=============================="
        self.log_message("Start"+self.__separator)

    def log_message(self, message):
        self.__log_file.writelines(message + "\r\n")
        print("Logged: " + message)

    def get_and_store_param(self, param_name):
        param_value = rospy.get_param(param_name)
        logger.log_message(param_name + " = " + str(param_value))

    def close(self):
        self.log_message("End"+self.__separator)
        self.__log_file.close()


class GazeboSim():
    def __init__(self):
        self.gazebo_process = None

    def start_gazebo(self):
        self.close_gazebo()
        self.gazebo_process = self.start_sub_process()

    def start_sub_process(self):
        package = 'ur3_moveit'
        node_name = 'start_sim.launch'

        command = "roslaunch {0} {1} spawn_obstacle:=true".format(
            package, node_name)

        process = subprocess.Popen(command, shell=True)

        rospy.wait_for_service("/arm_controller/query_state")
        rospy.wait_for_service("/move_group/plan_execution/set_parameters")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        time.sleep(1)
        state = process.poll()
        if state is None:
            rospy.loginfo("process is running fine")
            return process
        elif state < 0:
            rospy.loginfo("Process terminated with error")
        elif state > 0:
            rospy.loginfo("Process terminated without error")

    def kill_process_by_name(self, process_name):
        for proc in psutil.process_iter():
            if proc.name() == process_name:
                proc.kill()

    def close_gazebo(self):
        self.kill_process_by_name("gzserver")
        self.kill_process_by_name("gzclient")
        if (self.gazebo_process != None):
            os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)


def perform_single_measurement():
    robot.move_home()
    robot.clear_octomap()
    time.sleep(0.1)

    robot.move_to_A()
    robot.move_to_B()
    robot.move_to_A()

    robot.move_home()
    time.sleep(0.1)


if __name__ == "__main__":

    tested_parameters = ["/move_group/octomap_resolution",
                        "/move_group/max_safe_path_cost",
                        "/move_group/sense_for_plan/max_cost_sources",
                        "/move_group/sense_for_plan/max_look_attempts",
                        "/move_group/start_state_max_bounds_error",
                        "/move_group/trajectory_execution/allowed_start_tolerance",
                        "/move_group/ompl/simplify_solutions",
                        "/move_group/plan_execution/max_replan_attempts",
                        "/move_group/planning_scene_monitor/publish_planning_scene_hz",
                        "/robot_description_kinematics/manipulator/kinematics_solver",
                        "/robot_description_kinematics/manipulator/kinematics_solver_attempts",
                        "/robot_description_kinematics/manipulator/kinematics_solver_search_resolution",
                        "/robot_description_kinematics/manipulator/kinematics_solver_timeout",
                        "/robot_description_planning/default_object_padding",
                        "/robot_description_planning/default_robot_padding",
                        "/move_group/sensors"]  # save each sensor param

    logger = MeasurementLogger()

    for param_name in tested_parameters:
        logger.get_and_store_param(param_name)

    test_name = "Collision volumes: low poly - no obstacle"
    logger.log_message("Test name:" + test_name)

    # list all the parameters
    gazebo_sim = GazeboSim()
    gazebo_sim.start_gazebo()

    robot = robot_driver.RobotDriver()

    perform_single_measurement()

    gazebo_sim.close_gazebo()

    logger.close()
