#!/usr/bin/python

import time
import rospy
import os
import subprocess
import rospkg
import yaml
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from std_srvs.srv import Empty
from StringIO import StringIO
import rospkg
import argparse
import sys

import opti_logger as log
import pyswarm
import util_ros_process as ros_process
import robot_driver

current_script_path = os.path.dirname(os.path.realpath(__file__))
measurements_dir = os.path.join(current_script_path, "measurements")
obstacle_name = "obstacle"
test_repetitions = 30
sensors_param_name = "/move_group/sensors"

is_low_poly = True
is_rviz_online = False

tested_parameters = ["/move_group/manipulator/longest_valid_segment_fraction",
                     "/move_group/max_safe_path_cost",  # same?
                     "/move_group/octomap_resolution",
                     "/move_group/ompl/simplify_solutions",
                     "/move_group/plan_execution/max_replan_attempts",
                     "/move_group/sense_for_plan/discard_overlapping_cost_sources",
                     "/move_group/sense_for_plan/max_cost_sources",
                     "/move_group/sense_for_plan/max_look_attempts",
                     "/move_group/sense_for_plan/max_safe_path_cost",  # same?
                     "/move_group/planning_plugin",
                     "/robot_description_kinematics/manipulator/kinematics_solver",
                     "/robot_description_kinematics/manipulator/kinematics_solver_search_resolution",
                     "/robot_description_kinematics/manipulator/kinematics_solver_timeout",
                     "/robot_description_planning/default_robot_padding",
                     "/robot_description_planning/default_object_padding", ]


tested_planners = ["SBL", "EST", "LBKPIECE", "BKPIECE", "KPIECE", "RRT", "RRTConnect",
                   "RRTstar", "TRRT", "PRM", "PRMstar", "FMT",
                   # "BFMT", could not test
                   "PDST", "STRIDE", "BiTRRT",
                   # "LBTRRT", # could not test
                   "BiEST", "ProjEST", "LazyPRM", "LazyPRMstar", "SPARS", "SPARStwo"]


planned_poses = [
    robot_driver.joint_pose_A, robot_driver.joint_pose_B, robot_driver.joint_pose_A,
    #robot_driver.point_pose_A, robot_driver.point_pose_B, robot_driver.point_pose_A,
]

movements = ["Home -> A", "A -> B", "B -> A"]


plan_param_name = "/meas/plan"
exec_param_name = "/meas/execute"
success_param_name = "/meas/success"

def calculate_relative(value, min, max):
    if min > max:
        raise ArithmeticError()
    return (1 - ((value - min) / (max - min))) * 100

class ParametersTester:
    def __init__(self, watched_parameters, repetitions, is_obstacle_present, sim_env):
        self.watched_parameters = watched_parameters
        self.repetitions = repetitions
        rospy.on_shutdown(self.__shutdown_handler)
        self.environment = None
        self.is_obstacle_present = is_obstacle_present
        self.rob = None
        self.is_simulated_env = sim_env

    def measure_time(self, function, message="time spent"):
        start = time.time()
        success = function()
        if (not success):
            raise Exception("Function did not succeed")
        time_spent = time.time() - start
        return time_spent

    def start(self):
        ros_process.close_roscore()
        ros_process.start_roscore()
        self.environment = ros_process.Environment(
            is_obstacle_present=self.is_obstacle_present,
            is_workspace_limited=True,
            is_simulated=self.is_simulated_env)
        self.environment.start()

    def restart_move_group(self):
        robot_driver.restart_moveit_node()

    def wait_for_moveit_node(self):
        rospy.wait_for_service(robot_driver.clear_octomap_service)

    def end(self):
        self.environment.end()
        ros_process.close_roscore()

    def run_single_parameter_tests(self, tested_param_name, tested_param_value, planner_config_name="RRTConnect"):
        robot = robot_driver.RobotDriver()

        tested_param_value = str(tested_param_value)

        self.logger = log.MeasurementLogger(
            tested_param_name, tested_param_value, measurements_dir)
        self.postponed_logger = log.PostponedLogger(
            tested_param_name, tested_param_value, measurements_dir)

        test_name = tested_param_name + " : " + tested_param_value
        self.logger.log_message(test_name)
        self.logger.log_param("Planned repetitions", self.repetitions)
        self.postponed_logger.log_param(
            "Planned repetitions", self.repetitions)

        has_walls = self.check_space_limitation(robot.robot)

        robot.move_group.set_planner_id(planner_config_name)
        # robot.move_group.set_goal_joint_tolerance(1e-05)

        for cycle_num in range(1, self.repetitions + 1):
            
            self.logger.log_message("====== CYCLE %s ======" % cycle_num)
            self.postponed_logger.log_message(
                "====== CYCLE %s ======" % cycle_num)
            self.environment.reset()

            is_obstacle_present = obstacle_name in self.environment.get_sim_objects_names_list()
            self.log_env_params(planner_config_name, has_walls,
                                is_obstacle_present, robot)

            if (is_obstacle_present):
                obstacle_inital_pose = self.environment.get_model_pos(
                    obstacle_name)

            robot.clear_octomap()
            cycle_success = self.perform_movement_set(robot)

            obstacle_new_pose = self.environment.get_model_pos(obstacle_name)
            if (is_obstacle_present
                    and (obstacle_new_pose.pose != obstacle_inital_pose.pose or obstacle_new_pose.twist != obstacle_inital_pose.twist)):
                self.logger.log_error("Robot has moved the obstacle!")
                cycle_success = False

            self.logger.log_param("SUCCESSFUL CYCLE", cycle_success)
            self.logger.log_message("====== CYCLE %s END ======" % cycle_num)
            self.postponed_logger.log_message(
                "====== CYCLE %s END ======" % cycle_num)

            if (cycle_success):
                rospy.set_param(success_param_name, True)
                self.postponed_logger.write_messages()
            self.postponed_logger.dispose_messages()

        self.postponed_logger.close()
        self.logger.close()
        print("Param test finished")

    def log_env_params(self, planner_config_name, has_walls, is_obstacle_present, robot):
        self.logger.log_section("Params")

        self.logger.log_param("Planner", planner_config_name)
        self.logger.log_param("Invisible walls", has_walls)
        self.logger.log_param("Goal joint tolerance",
                              robot.move_group.get_goal_joint_tolerance())
        self.logger.log_param(
            "Goal orientation tolerance", robot.move_group.get_goal_orientation_tolerance())
        self.logger.log_param("Goal position tolerance",
                              robot.move_group.get_goal_position_tolerance())
        self.logger.log_param("Collision model is low poly", is_low_poly)
        self.logger.log_param("RViz", is_rviz_online)
        self.logger.log_param("Obstacle", is_obstacle_present)
        self.__log_current_ros_params()

    def check_space_limitation(self, robot):
        link_names = robot.get_link_names()
        return "top_wall" in link_names or "left_wall" in link_names or "back_wall" in link_names

    def __log_current_ros_params(self):
        for param_name in self.watched_parameters:
            self.logger.log_param(param_name, rospy.get_param(param_name))

        yaml_params = [sensors_param_name]
        for param_name in yaml_params:
            self.save_yaml_param_values(param_name)

    def save_yaml_param_values(self, param_name):
        str_io = StringIO(rospy.get_param(param_name))
        param_items = yaml.load(str_io)
        for index in range(0, len(param_items)):
            for key in param_items[index]:
                self.logger.log_param(
                    param_name+str(index)+"/"+key, param_items[index][key])

    def perform_movement_set(self, robot):
        self.logger.log_section("Time Measurement")
        
        success = True
        if (not robot.move_home()):
            success = False
        time.sleep(0.1)

        for pose, label in zip(planned_poses, movements):
            robot.set_target_pose(pose)
            try:
                plan_time = self.measure_time(robot.perform_planning) 
                self.logger.log_message(
                    "Planning time to %s: %s" % (pose.name, plan_time))
                self.postponed_logger.log_message(
                    "Planning time to %s: %s" % (pose.name, plan_time))
                rospy.set_param(plan_param_name+label, plan_time)
                execution_time = self.measure_time(robot.execute_planned_sync)
                self.logger.log_message(
                    "Execution time to %s: %s" % (pose.name, execution_time))
                self.postponed_logger.log_message(
                    "Execution time to %s: %s" % (pose.name, execution_time))
                rospy.set_param(exec_param_name+label, execution_time)
            except Exception as ex:
                self.logger.log_error("Movement to %s failed" % pose.name)
                success = False
                break

        if (not robot.move_home()):
            success = False
        time.sleep(0.1)
        return success

    def __shutdown_handler(self):
        pass

    def get_sensors_config(self, point_subsample=None):
        with open(os.path.join(rospkg.RosPack().get_path("ur3_moveit"), "config", "sensors_3d.yaml")) as file:
            config = yaml.load(file)["sensors"]
            if point_subsample != None:
                config[0]["point_subsample"] = point_subsample
            return config


if __name__ == "__main__":

    raise EnvironmentError

    tester = ParametersTester(tested_parameters, 30, is_obstacle_present=True, sim_env=True)
    tester.start()
    
    #default
    # octomap_res = 0.02
    # point_subs = 1
    # segment = 0.005
    # planner = "RRTConnect"

    #Optimized
    octomap_res = 0.0678
    point_subs = 125
    segment = 0.0056
    planner = "BiTRRT"
    robot_driver.set_dynamic_params([octomap_res, point_subs, segment])
    
    tester.run_single_parameter_tests("Simulation B2", "Optimized LP " + planner, planner)

    # for planner in tested_planners:
        # tester.run_single_parameter_tests("Planner", planner, planner_config_name=planner)
    #"Oct res "+octomap_res+"; subsample "+point_subs+"; longest valid segment "+segment+"; Low poly; "+planner
    tester.end()
