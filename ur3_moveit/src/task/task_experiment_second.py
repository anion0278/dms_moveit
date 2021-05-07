#!/usr/bin/env python

import rospy
import sys
import numpy as np
from ur_msgs.srv import *
from icecream import ic
import setproctitle
setproctitle.setproctitle("DMS Task Commander - Experiment 2")

import sys, os
sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )
from utils import util_keyboard as keyboard

import robot_driver as r
import task_commander

screw_poses = [
    r.get_joint_pose_from_deg("Screw - left",[12, -82, 126, -134, -89, -1]),
    r.get_joint_pose_from_deg("Screw - center",[56, -96, 141, -135, -90, -33]),
    r.get_joint_pose_from_deg("Screw - right",[124, -81, 124, -133, -90, -3])]

camera_poses = [
    r.get_joint_pose_from_deg("Camera - left",[11, -87, 99, -101, -90, 0]),
    r.get_joint_pose_from_deg("Camera - center",[57, -110, 109, -88, -90, -33]),
    r.get_joint_pose_from_deg("Camera - right",[125, -87, 99, -101, -90, 0])]

appro_toolbox_poses = [
    r.get_joint_pose_from_deg("Appro-Toolbox - left",[-25, -101, 128, -117, -90, 0]),
    r.get_joint_pose_from_deg("Appro-Toolbox - right",[153, -99, 123, -114, -90, 0])]

toolbox_poses = [
    r.get_joint_pose_from_deg("Toolbox - left",[-25, -94, 135, -132, -90, 0]),
    r.get_joint_pose_from_deg("Toolbox - right",[155, -94, 135, -132, -90, 0])]


def run_second_experiment_sequence(commander):
    normal_speed = 0.6
    normal_acc = 0.5
    
    print("Starting experiment 2 at %s speed..." % normal_speed)
    commander.robot_driver.set_speed_params(normal_acc, normal_speed)
    commander.move_to(init_pose)
    key = keyboard.choose_option_by_key({"y": "Start the experiment"})
    rospy.sleep(3)
    goal_sequence = np.arange(3)
    screw_numbers = np.array([1,0,1])
    # toolbox_sequences = np.array([0,1,1], [1,0,1], [0,1,1])
    np.random.shuffle(goal_sequence)
    np.random.shuffle(screw_numbers)
    for i, goal in zip(range(len(goal_sequence)), goal_sequence):
        print("Box: %s" % i)
        commander.move_to(camera_poses[goal])
        rospy.sleep(2)
        for _ in range(screw_numbers[goal]):
            import random
            # toolbox_num = random.randint(0, len(toolbox_poses)-1)
            # commander.move_to(appro_toolbox_poses[toolbox_num])
            # commander.move_to(toolbox_poses[toolbox_num])
            # print("Taking a screw")
            rospy.sleep(0.3)
            # commander.move_to(appro_toolbox_poses[toolbox_num])
            # commander.move_to(camera_poses[goal])
            commander.move_to(screw_poses[goal])
            print("Screw-driwing")
            rospy.sleep(1.5)
            commander.move_to(camera_poses[goal])
    commander.move_to(init_pose)


if __name__ == "__main__":
    commander = task_commander.get_task_commander()

    # init_pose = r.get_joint_pose_from_deg("Initial Pose",[90,-90,90,-90,-90,0])
    init_pose = r.get_joint_pose_from_deg("Initial Pose",[90,-90,0,-90,-90,0])
    run_second_experiment_sequence(commander)