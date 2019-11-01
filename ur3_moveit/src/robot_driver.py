#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
import time

home_position = "Home"
group_name = "manipulator"
clear_octomap_service = "/clear_octomap"


class RobotJointPose():
    def __init__(self, joint_angles, name):
        self.joint_angles = joint_angles
        self.name = name


pose_A = RobotJointPose([-0.45161887833653935,
                         -0.8200713084170896,
                         0.9261690554869411,
                         -1.6652095580852606,
                         -1.5501685915793058,
                         1.0543118993180052],
                        "Point A")


pose_B = RobotJointPose([1.112419230110138,
                         -0.8036786161659402,
                         0.8430760165715983,
                         -1.6316082500996831,
                         -1.5601357714877526,
                         1.3836915426267309],
                        "Point B")

class ExecutionStatus:
    def __init__(self):
        pass

class RobotDriver:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_python_driver', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group_name)

        rospy.wait_for_service(clear_octomap_service)
        self.__clear_octomap_service = rospy.ServiceProxy(
            clear_octomap_service, Empty)
        self.clear_octomap()

        print("Robot current state")
        print self.robot.get_current_state()
        print self.group.get_current_pose()

    def clear_octomap(self):
        print("Octomap clearing...")
        # the service type is Empty, that is why there are no arguments
        self.__clear_octomap_service()

    # returns measured time - for planning and for execution
    def move_to_joint_pose(self, pose):
        self.set_joint_target(pose.joint_angles)
        self.perform_planning()
        self.execute_planned_sync()

    def set_joint_target(self, pose):
        self.group.set_joint_value_target(pose.joint_angles)

    def perform_planning(self):
        plan = self.group.plan()
        success = plan[0]
        if (not success):
            print("Could not plan the movement! Plan: " + str(plan))
        return success

    def execute_planned_sync(self):
        success = self.group.go(wait=True)
        if (not success):
            print("Could not execute the movement!")
        return success

    def move_home(self):
        print("Moving to Home position")
        self.group.set_named_target(home_position)
        success = self.perform_planning()
        success = self.execute_planned_sync() and success
        return success

    def move_from_A_to_B(self):
        self.move_to_joint_pose(pose_A)
        self.move_to_joint_pose(pose_B)

    def execute_cyclic_movement(self, repetitions):
        self.move_home()

        print("Moving in cycles")
        for index in range(0, repetitions - 1):
            self.move_from_A_to_B()

        self.move_home()


if __name__ == "__main__":
    robot_driver = RobotDriver()
    robot_driver.execute_cyclic_movement(5)
