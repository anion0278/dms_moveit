#!/usr/bin/python

import time
import robot_driver
import rospy
import os
import subprocess
import roslaunch
import rospkg
import yaml
import gazebo_sim
import logger as log
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from std_srvs.srv import Empty
from StringIO import StringIO

current_script_path = os.path.dirname(os.path.realpath(__file__))
measurements_dir = os.path.join(current_script_path, "measurements")
obstacle_name = "obstacle"

is_low_poly = True
is_rviz_online = False

class ParameterTester:
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

    def run_planners_tests(self):
        self.gazebo_sim = gazebo_sim.GazeboSim(
            is_obstacle_present=self.is_obstacle_present, 
            is_workspace_limited=True)
        self.gazebo_sim.start_gazebo()
        robot = robot_driver.RobotDriver()

        planners = [
            "SBL", "EST", "LBKPIECE", "BKPIECE", "KPIECE", "RRT", "RRTConnect", "RRTstar", "TRRT", "PRM", "PRMstar", "FMT",
            # "BFMT", could not test
            "PDST", "STRIDE", "BiTRRT",
                    # "LBTRRT", # could not test
            "BiEST", "ProjEST", "LazyPRM", "LazyPRMstar", "SPARS", "SPARStwo"
        ]

        for planner_name in planners:
            self.__run_tests(
                robot, "Planner", planner_name, planner_name)

        self.gazebo_sim.close_gazebo()

    def run_single_parameters_tests(self, param_name, param_value):
        self.gazebo_sim = gazebo_sim.GazeboSim(
            is_obstacle_present=self.is_obstacle_present, 
            is_workspace_limited=True)
        self.gazebo_sim.start_gazebo()
        robot = robot_driver.RobotDriver()

        self.__run_tests(robot, param_name, param_value)

        self.gazebo_sim.close_gazebo()

    def __run_tests(self, robot_driver, tested_param_name, tested_param_value, planner_config_name="RRTConnect"):
        tested_param_value = str(tested_param_value)

        self.logger = log.MeasurementLogger(
            tested_param_name, tested_param_value, measurements_dir)
        self.postponed_logger = log.PostponedLogger(
            tested_param_name, tested_param_value, measurements_dir)

        test_name = tested_param_name + " : " + tested_param_value
        self.logger.log_message(test_name)
        self.logger.log_param("Planned repetitions", self.repetitions)

        has_walls = self.CheckLimitingWalls(robot_driver.robot)

        robot_driver.group.set_planner_id(planner_config_name)

        for cycle_num in range(1, self.repetitions + 1):
            self.logger.log_message("====== CYCLE %s ======" % cycle_num)
            self.postponed_logger.log_message(
                "====== CYCLE %s ======" % cycle_num)
            self.gazebo_sim.reset_sim()

            robot_driver.group.set_goal_position_tolerance(0.002)
            # robot_driver.group.set_goal_orientation_tolerance()

            self.logger.log_section("Params")
            is_obstacle_present = obstacle_name in self.gazebo_sim.get_sim_objects_names_list()
            self.logger.log_param("Planner", planner_config_name)
            self.logger.log_param("Invisible walls", has_walls)
            self.logger.log_param("Goal joint tolerance",
                                  robot_driver.group.get_goal_joint_tolerance())
            self.logger.log_param(
                "Goal orientation tolerance", robot_driver.group.get_goal_orientation_tolerance())
            self.logger.log_param("Goal position tolerance",
                                  robot_driver.group.get_goal_position_tolerance())
            self.logger.log_param("Collision model is low poly", is_low_poly)
            self.logger.log_param("RViz", is_rviz_online)
            self.logger.log_param("Obstacle", is_obstacle_present)

            self.save_current_ros_params()

            if (is_obstacle_present):
                obstacle_inital_pose = self.gazebo_sim.get_model_pos(
                    obstacle_name)

            self.logger.log_section("Time Measurement")
            cycle_success = self.perform_single_movement_set(robot_driver)

            actual_pose = self.gazebo_sim.get_model_pos(obstacle_name)
            if (is_obstacle_present
                    and (actual_pose.pose != obstacle_inital_pose.pose or actual_pose.twist != obstacle_inital_pose.twist)):
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

    def CheckLimitingWalls(self, robot):
        link_names = robot.get_link_names()
        return "top_wall" in link_names or "left_wall" in link_names or "back_wall" in link_names

    def save_current_ros_params(self):
        for param_name in self.watched_parameters:
            self.logger.log_param(param_name, rospy.get_param(param_name))

        yaml_params = ["/move_group/sensors"]
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

        planned_poses = [robot_driver.pose_A,
                         robot_driver.pose_B, robot_driver.pose_A]

        for pose in planned_poses:
            self.logger.log_param(pose.name, "Joint target")
            robot.set_joint_target(pose)
            try:
                plan_time = self.measure_time(robot.perform_planning)
                self.logger.log_message(
                    "Planning time to %s: %s" % (pose.name, plan_time))
                self.postponed_logger.log_message(
                    "Planning time to %s: %s" % (pose.name, plan_time))
                execution_time = self.measure_time(robot.execute_planned_sync)

                # self.logger.log_message(
                #     "Position: " + str(robot.robot.get_current_pose()))

                # TODO remove typo!!!
                self.logger.log_message(
                    "Exection time to %s: %s" % (pose.name, execution_time))
                self.postponed_logger.log_message(
                    "Exection time to %s: %s" % (pose.name, execution_time))
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


class ParameterWithOptions:
    def __init__(self, param_name, value_options):
        self.param_name = param_name
        self.value_options = value_options


if __name__ == "__main__":

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
                         "/robot_description_kinematics/manipulator/kinematics_solver_attempts",
                         "/robot_description_kinematics/manipulator/kinematics_solver_search_resolution",
                         "/robot_description_kinematics/manipulator/kinematics_solver_timeout",

                         "/robot_description_planning/default_robot_padding",
                         "/robot_description_planning/default_object_padding", ]

    tester = ParameterTester(tested_parameters, 30, is_obstacle_present=True)

    # tester.run_planners_tests()

    tester.run_single_parameters_tests("Path simplification", "Disabled")
    # group.allow_replanning
    # group.allow_looking
