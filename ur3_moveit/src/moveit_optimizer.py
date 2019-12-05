#!/usr/bin/python
import os
import sys
import subprocess
import ros_process
import moveit_tester
import rospy
import time
import logger
import pyswarm
from statistics import mean
from collections import OrderedDict

current_script_path = os.path.dirname(os.path.realpath(__file__))

repetitions = 10

actions_min_max_vals = {
    moveit_tester.plan_param_name + moveit_tester.movements[0]: (0.074, 0.33),
    moveit_tester.exec_param_name + moveit_tester.movements[0]: (2.8, 9.59),
    moveit_tester.plan_param_name + moveit_tester.movements[1]: (0.051, 0.344),
    moveit_tester.exec_param_name + moveit_tester.movements[1]: (2.685, 9.94),
    moveit_tester.plan_param_name + moveit_tester.movements[2]: (0.05, 0.4),
    moveit_tester.exec_param_name + moveit_tester.movements[2]: (2.68, 10),
}

pso_cycle = 0
pso_swarm_size = 20
logger = logger.MeasurementLogger("PSO", "Values", current_script_path)


def get_overal_preformance(parameters):
    global pso_cycle
    if pso_cycle % pso_swarm_size == 0:
        logger.log_param("++++ New population", pso_cycle // pso_swarm_size)
    pso_cycle += 1

    logger.log_param("**** Particle", pso_cycle)
    logger.log_param("==== New values", ["%.4f" % item for item in parameters])
    logger.flush()

    rospy.set_param("/move_group/octomap_resolution", float(parameters[0]))
    rospy.set_param(moveit_tester.sensors_param_name, tester.get_sensors_config(
        point_subsample=int(parameters[1])))
    rospy.set_param(
        "/move_group/manipulator/longest_valid_segment_fraction", float(parameters[2]))

    averages = OrderedDict()
    averages["success"] = 0

    tester.apply_new_move_group_params()

    for cycle in range(repetitions):
        # tester.gazebo_sim.start()

        logger.log_param("---- Cycle", cycle)
        rospy.set_param(moveit_tester.success_param_name, False)

        for movement in moveit_tester.movements:
            plan_name = moveit_tester.plan_param_name+movement
            exec_name = moveit_tester.exec_param_name+movement
            rospy.set_param(plan_name, 0)
            rospy.set_param(exec_name, 0)

        try:
            tester.run_single_parameter_tests(
                "Optimization", "PSO %s" % parameters,
                planner_config_name="BiTRRT")
        except Exception as ex:
            logger.log_message("TIMEOUT")
            tester.gazebo_sim.reset_sim()
            tester.apply_new_move_group_params()
            tester.gazebo_sim.reset_sim()
            time.sleep(3)
            tester.run_single_parameter_tests(
                "Optimization", "PSO %s" % parameters,
                planner_config_name="BiTRRT")
            pass

        k = 10
        if rospy.get_param(moveit_tester.success_param_name):
            averages["success"] += 1
            k = 1

        for movement in moveit_tester.movements:
            plan_name = moveit_tester.plan_param_name+movement
            exec_name = moveit_tester.exec_param_name+movement
            if cycle == 0:
                averages[plan_name] = rospy.get_param(plan_name)
                averages[exec_name] = rospy.get_param(exec_name)

            averages[plan_name] = k * mean(
                [rospy.get_param(plan_name), averages[plan_name]])
            averages[exec_name] = k * mean(
                [rospy.get_param(exec_name), averages[exec_name]])

        logger.log_param("averages", averages)

    for movement in moveit_tester.movements:
        plan_name = moveit_tester.plan_param_name+movement
        exec_name = moveit_tester.exec_param_name+movement

        if averages[plan_name] == 0 or averages[exec_name] == 0:
            averages[plan_name] = averages[exec_name] = 10000

        averages[plan_name] = moveit_tester.calculate_relative(
            averages[plan_name],
            actions_min_max_vals[plan_name][0],
            actions_min_max_vals[plan_name][1])
        averages[exec_name] = moveit_tester.calculate_relative(
            averages[exec_name],
            actions_min_max_vals[exec_name][0],
            actions_min_max_vals[exec_name][1])

    averages["success"] /= repetitions
    total_result = - mean(averages.values())

    logger.log_param("Total result", total_result)
    logger.flush()

    return total_result


if __name__ == "__main__":
    # octomap res, points subsample, valid segment fraction
    lower_boundaries = [0.01, 0, 0.001]
    upper_boundaries = [0.1, 100, 0.01]

    tester = moveit_tester.ParametersTester(
        moveit_tester.tested_parameters, 1, is_obstacle_present=True)
    tester.start()

    xopt, fopt = pyswarm.pso(get_overal_preformance,
                             lower_boundaries,
                             upper_boundaries,
                             swarmsize=pso_swarm_size, debug=True, maxiter=50)
    print (xopt)
    tester.end()

    # for i, item in enumerate(x):
    # ...     print(i, item)
