#!/usr/bin/python
import os
import sys
import subprocess
import rospy
import time
from statistics import mean
from collections import OrderedDict
import pyswarm

import sys, os
sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )
from task import robot_driver
import opti_logger
import opti_moveit_tester


current_script_path = os.path.dirname(os.path.realpath(__file__))

repetitions = 5

actions_min_max_vals = {
    opti_moveit_tester.plan_param_name + opti_moveit_tester.movements[0]: (0.074, 0.33),
    opti_moveit_tester.exec_param_name + opti_moveit_tester.movements[0]: (2.8, 9.59),
    opti_moveit_tester.plan_param_name + opti_moveit_tester.movements[1]: (0.051, 0.344),
    opti_moveit_tester.exec_param_name + opti_moveit_tester.movements[1]: (2.685, 9.94),
    opti_moveit_tester.plan_param_name + opti_moveit_tester.movements[2]: (0.05, 0.4),
    opti_moveit_tester.exec_param_name + opti_moveit_tester.movements[2]: (2.68, 10),
}

__pso_cycle = 0
pso_swarm_size = 20
logger = opti_logger.MeasurementLogger("PSO", "Values", current_script_path)

planner = "RRTConnect" #"BiTRRT" 

def get_overal_preformance(parameters):
    global __pso_cycle
    if __pso_cycle % pso_swarm_size == 0:
        logger.log_param("++++ New population", __pso_cycle // pso_swarm_size)
    __pso_cycle += 1

    logger.log_param("**** Particle", __pso_cycle)
    logger.log_param("==== New values", ["%.4f" % item for item in parameters])
    logger.flush()

    robot_driver.set_dynamic_params(parameters)

    averages = get_averages_container()

    for cycle in range(repetitions):
        logger.log_param("---- Cycle", cycle)

        initialize_meas_params()
        tester.run_single_parameter_tests(
            "Optimization", "PSO NEW %s" % parameters,
            planner_config_name=planner)

        update_averages(cycle, averages)
        logger.log_param("averages", averages)

    total_result = calculate_fittness(averages)

    logger.log_param("Total result", total_result)
    logger.flush()

    return total_result


def calculate_fittness(averages):
    for movement in opti_moveit_tester.movements:
        plan_name = opti_moveit_tester.plan_param_name+movement
        exec_name = opti_moveit_tester.exec_param_name+movement

        if averages[plan_name] == 0 or averages[exec_name] == 0:
            averages[plan_name] = averages[exec_name] = 1000

        averages[plan_name] = opti_moveit_tester.calculate_relative(
            averages[plan_name],
            actions_min_max_vals[plan_name][0],
            actions_min_max_vals[plan_name][1])

        averages[exec_name] = opti_moveit_tester.calculate_relative(
            averages[exec_name],
            actions_min_max_vals[exec_name][0],
            actions_min_max_vals[exec_name][1])

    averages["success"] /= repetitions
    total_result = - mean(averages.values())
    return total_result


def update_averages(cycle, averages):
    k = 10
    if rospy.get_param(opti_moveit_tester.success_param_name):
        averages["success"] += 1
        k = 1   

    for movement in opti_moveit_tester.movements:
        plan_name = opti_moveit_tester.plan_param_name+movement
        exec_name = opti_moveit_tester.exec_param_name+movement
        if cycle == 0:
            averages[plan_name] = rospy.get_param(plan_name)
            averages[exec_name] = rospy.get_param(exec_name)

        averages[plan_name] = k * mean([rospy.get_param(plan_name), averages[plan_name]])
        averages[exec_name] = k * mean([rospy.get_param(exec_name), averages[exec_name]])


def get_averages_container():
    averages = OrderedDict()
    averages["success"] = 0
    return averages


def initialize_meas_params():
    rospy.set_param(opti_moveit_tester.success_param_name, False)
    for movement in opti_moveit_tester.movements:
        plan_name = opti_moveit_tester.plan_param_name+movement
        exec_name = opti_moveit_tester.exec_param_name+movement
        rospy.set_param(plan_name, 0)
        rospy.set_param(exec_name, 0)


if __name__ == "__main__":

    # octomap res, points subsample, valid segment fraction
    lower_boundaries = [0.01, 1, 0.001]
    upper_boundaries = [0.1, 200, 0.01]

    tester = opti_moveit_tester.ParametersTester(opti_moveit_tester.tested_parameters, 1, is_obstacle_present=True, sim_env=True)
    tester.start()

    # just initialization
    robot_driver.set_dynamic_params([0.05, 1, 0.005])

    xopt, fopt = pyswarm.pso(get_overal_preformance,
                             lower_boundaries,
                             upper_boundaries,
                             omega = 0.8, phip = 1.2, phig = 1.4,
                             swarmsize=pso_swarm_size, debug=True, maxiter=1000)
    print (xopt)
    tester.end()

    # for i, item in enumerate(x):
    # ...     print(i, item)

    # input = sys.stdin.readline()
    # while not input.strip() in ['y', 'n']:
    #     input = sys.stdin.readline()
