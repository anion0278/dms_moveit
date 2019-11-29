#!/usr/bin/python

import time
import robot_driver
import rospy
import os
import subprocess
import rospkg
import yaml
import ros_process
import logger as log
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from std_srvs.srv import Empty
from StringIO import StringIO
import rospkg
import argparse, sys

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


class ParametersTester:
    def __init__(self, watched_parameters, repetitions, is_obstacle_present):
        self.watched_parameters = watched_parameters
        self.repetitions = repetitions
        rospy.on_shutdown(self.__shutdown_handler)
        self.gazebo_sim = None
        self.is_obstacle_present = is_obstacle_present

    def measure_time(self, function, message="time spent"):
        start = time.time()
        success = function()
        if (not success):
            raise Exception("Function did not succeed")
        time_spent = time.time() - start
        return time_spent

    def run_single_parameter_tests(self, param_name, param_value, segment_fraction=None, octomap_resolution=None, sensors_config=None, planner_config_name="RRTConnect"):
        self.gazebo_sim = ros_process.GazeboSim(
            is_obstacle_present=self.is_obstacle_present,
            is_workspace_limited=True)

        # if (sensors_config != None):
        #     rospy.set_param(sensors_param_name, sensors_config)

        # additional_params = ""
        # if (segment_fraction != None):
        #     additional_params += " segment_fraction:="+str(segment_fraction)

        # if (octomap_resolution != None):
        #     additional_params += " octomap_resolution:=" + \
        #         str(octomap_resolution)

        self.gazebo_sim.start()
        robot = robot_driver.RobotDriver()

        rospy.set_param("sim_ok", True)
        self.__run_tests(robot, param_name, param_value, planner_config_name)

        self.gazebo_sim.close_gazebo()

    def __run_tests(self, robot_driver, tested_param_name, tested_param_value, planner_config_name):
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

        has_walls = self.check_space_limitation(robot_driver.robot)

        robot_driver.move_group.set_planner_id(planner_config_name)

        for cycle_num in range(1, self.repetitions + 1):

            rospy.set_param("/move_group/octomap_resolution", 0.02 / cycle_num)
            ros_process.kill_process_by_name("move_group")

            rospy.wait_for_service("/move_group/plan_execution/set_parameters")
            

            self.logger.log_message("====== CYCLE %s ======" % cycle_num)
            self.postponed_logger.log_message(
                "====== CYCLE %s ======" % cycle_num)
            self.gazebo_sim.reset_sim()

            is_obstacle_present = obstacle_name in self.gazebo_sim.get_sim_objects_names_list()
            self.log_sim_params(planner_config_name, has_walls,
                                is_obstacle_present, robot_driver)

            if (is_obstacle_present):
                obstacle_inital_pose = self.gazebo_sim.get_model_pos(
                    obstacle_name)

            self.logger.log_section("Time Measurement")
            cycle_success = self.perform_single_movement_set(robot_driver)

            obstacle_new_pose = self.gazebo_sim.get_model_pos(obstacle_name)
            if (is_obstacle_present
                    and (obstacle_new_pose.pose != obstacle_inital_pose.pose or obstacle_new_pose.twist != obstacle_inital_pose.twist)):
                self.logger.log_error("Robot has moved the obstacle!")
                cycle_success = False

            self.logger.log_param("SUCCESSFUL CYCLE", cycle_success)
            self.logger.log_message("====== CYCLE %s END ======" % cycle_num)
            self.postponed_logger.log_message(
                "====== CYCLE %s END ======" % cycle_num)

            if (cycle_success):
                self.postponed_logger.write_messages()
            self.postponed_logger.dispose_messages()

        self.postponed_logger.close()
        self.logger.close()

    def log_sim_params(self, planner_config_name, has_walls, is_obstacle_present, robot_driver):
        self.logger.log_section("Params")

        self.logger.log_param("Planner", planner_config_name)
        self.logger.log_param("Invisible walls", has_walls)
        self.logger.log_param("Goal joint tolerance",
                              robot_driver.move_group.get_goal_joint_tolerance())
        self.logger.log_param(
            "Goal orientation tolerance", robot_driver.move_group.get_goal_orientation_tolerance())
        self.logger.log_param("Goal position tolerance",
                              robot_driver.move_group.get_goal_position_tolerance())
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

    def perform_single_movement_set(self, robot):
        robot.clear_octomap()
        success = True
        if (not robot.move_home()):
            success = False
        time.sleep(0.1)

        planned_poses = [
            robot_driver.joint_pose_A, robot_driver.joint_pose_B, robot_driver.joint_pose_A,
            #robot_driver.point_pose_A, robot_driver.point_pose_B, robot_driver.point_pose_A,
        ]

        for pose in planned_poses:
            robot.set_target_pose(pose)
            try:
                plan_time = self.measure_time(robot.perform_planning)
                self.logger.log_message(
                    "Planning time to %s: %s" % (pose.name, plan_time))
                self.postponed_logger.log_message(
                    "Planning time to %s: %s" % (pose.name, plan_time))
                execution_time = self.measure_time(robot.execute_planned_sync)
                self.logger.log_message(
                    "Execution time to %s: %s" % (pose.name, execution_time))
                self.postponed_logger.log_message(
                    "Execution time to %s: %s" % (pose.name, execution_time))
            except Exception as ex:
                self.logger.log_error("Movement to %s failed" % pose.name)
                success = False
                break

        if (not robot.move_home()):
            success = False
        time.sleep(0.1)
        return success

    def __shutdown_handler(self):
        self.gazebo_sim.close_gazebo()
        pass


def get_sensors_config(point_subsample=None):
    with open(os.path.join(rospkg.RosPack().get_path("ur3_moveit"), "config", "sensors_3d.yaml")) as file:
        config = yaml.load(file)["sensors"]
        if point_subsample != None:
            config[0]["point_subsample"] = point_subsample
        return config


def get_statistics_for_params():
    tester = ParametersTester(
        tested_parameters, 10, is_obstacle_present=True)
      
        # TODO Measure the length of trajectory!
    tester.run_single_parameter_tests(
        "Joint model state", "Enforced",
        planner_config_name="BiTRRT",
        sensors_config=get_sensors_config(point_subsample=20),
        segment_fraction=0.005,
        octomap_resolution=0.02)

if __name__ == "__main__":

    ros_process.start_roscore()
    get_statistics_for_params()
    ros_process.close_roscore()
