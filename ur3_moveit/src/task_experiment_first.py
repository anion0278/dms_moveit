#!/usr/bin/env python

import rospy
import sys
from ur_msgs.srv import *
from icecream import ic
import setproctitle
setproctitle.setproctitle("DMS Task Commander - Experiment 1")

import robot_driver as r
import util_keyboard as keyboard
import task_commander

def run_first_experiment_sequence(commander, repetitions):
    normal_speed = 0.9
    test_speed = 0.05
    print("Starting experiment 1 at %s speed..." % test_speed)
    commander.robot_driver.set_speed_params(normal_speed, normal_speed)
    commander.move_to(init_pose)
    import random
    for i in range(1, repetitions+1):
        print("Cycle: %s" % i)
        key = keyboard.choose_option_by_key({"y": "Start cycle"})
        random_pos_num = random.randint(0, 4) # inclusiveness was ambiguous - result will be 0..4
        print("Moving to random goal: %s " % (random_pos_num + 1))
        commander.robot_driver.set_speed_params(test_speed, test_speed)
        if key == "y":
            commander.move_to(goals[random_pos_num], enable_preempting=True)
        commander.robot_driver.set_speed_params(normal_speed, normal_speed)
        commander.move_to(init_pose)
    commander.robot_driver.set_speed_params(normal_speed, normal_speed)
    commander.move_to(init_pose)


if __name__ == "__main__":
    commander = task_commander.get_task_commander()

    init_pose = r.get_joint_pose_from_deg("Initial Pose",[90,-90,0,-90,-90,0])
    goals = [
        # r.get_joint_pose_from_deg("Goal 1 - Top Left",[4, -49, 96, -137, -90, -89]),
        r.get_joint_pose_from_deg("Goal 1 - Top Left",[10, -49, 96, -137, -90, -89]), # lower
        # r.get_joint_pose_from_deg("Goal 2 - Bottom Left",[35, -14, 22, -98, -90, -58]), # first
        r.get_joint_pose_from_deg("Goal 2 - Bottom Left",[55, -35, 61, -118, -90, -58]), 
        # r.get_joint_pose_from_deg("Goal 2 - Bottom Left",[29, -30, 17, -151, -90, -6]), # tyce
        r.get_joint_pose_from_deg("Goal 3 - Center",[67, -50, 133, -173, -90, -23]),
        # r.get_joint_pose_from_deg("Goal 4 - Top Right",[143, -39, 96, -147, -90, 53]),
        r.get_joint_pose_from_deg("Goal 4 - Top Right",[136, -39, 96, -147, -90, 53]), # lower
        # r.get_joint_pose_from_deg("Goal 5 - Bottom Right",[121, -1, 10, -98, -90, 31]),
        r.get_joint_pose_from_deg("Goal 5 - Bottom Right",[100, -23, 57, -123, -90, 31]),
        # r.get_joint_pose_from_deg("Goal 5 - Bottom Right",[122, -33, 8, -110, -62, -3]), #tyce
    ]

    phase_msg_format = "{}) ############################# {} #############################" # todo limit to 77 symbols

    print(phase_msg_format.format(1, "Goals demo (Full speed!)"))
    key = keyboard.choose_option_by_key({"y": "Start goal demo", "n": "Skip goal demo"}, use_delay=True)
    if key == "y":
        speed = 0.9
        first_experiment_poses_demo = []
        for i in range(0, len(goals)):
            first_experiment_poses_demo.append(init_pose)
            first_experiment_poses_demo.append(goals[i])
        first_experiment_poses_demo.append(init_pose)
        commander.robot_driver.set_speed_params(speed, speed)
        commander.demo_task_sequence(first_experiment_poses_demo, repetitions=1, move_to_home = False)
        print("Goals demo finished")

    demo_repetitions = 3
    print(phase_msg_format.format(2, "HMI demo (%sx)" % demo_repetitions))
    key = keyboard.choose_option_by_key({"y": "Run HMI training (%sx)" % demo_repetitions, "n": "Skip HMI training"}, use_delay=True)
    if key == "y":
        run_first_experiment_sequence(commander, demo_repetitions)
        print("First-time HMI demo finished")

    print(phase_msg_format.format(3, "Experiment 1"))
    run_first_experiment_sequence(commander, 5)
    print("Experiment 1 finished !")