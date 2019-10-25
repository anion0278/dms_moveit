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


def measure_time(function, message="time spent"):
    start = time.time()
    function()
    time_spent = time.time() - start
    print (message + ": " + str(time_spent))
    return time_spent

class RobotDriver:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_python_driver', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group_name)
        
        rospy.wait_for_service(clear_octomap_service)
        self.__clearOctomapService = rospy.ServiceProxy(clear_octomap_service, Empty)
        self.clear_octomap()

        print("Robot current state")
        print self.robot.get_current_state()
        print self.group.get_current_pose()

    def clear_octomap(self):
        self.__clearOctomapService() # the service type is Empty, that is why there are no arguments

    def move_to_joint_pose(self, pose):
        self.group.set_joint_value_target(pose.joint_angles)
        measure_time(self.group.plan, "planning " + pose.name)
        measure_time(lambda: self.group.go(wait=True), "movement execution " + pose.name)

    def move_home(self):
        print("Moving to Home position")
        self.group.set_named_target(home_position)
        self.group.plan()
        self.group.go(wait=True)

    def move_from_A_to_B(self):
        self.move_to_A()
        self.move_to_B()

    def move_to_A(self):
        self.move_to_joint_pose(pose_A)

    def move_to_B(self):
        self.move_to_joint_pose(pose_B)

    def execute_cyclic_movement(self):
        self.move_home()

        print("Moving in cycles")
        for index in range(0, 4):
            self.move_from_A_to_B()

        self.move_home()


if __name__ == "__main__":
    robot_driver = RobotDriver()
    robot_driver.execute_cyclic_movement()

# group.set_goal_orientation_tolerance(10)
# group.set_goal_position_tolerance(10)

# waypoints = []

# scale = 1
# wpose1 = group.get_current_pose().pose
# print ("Current pose: %s" % wpose1)
# wpose2 = copy.deepcopy(wpose1)
# wpose3 = copy.deepcopy(wpose1)
# wpose4 = copy.deepcopy(wpose1)

# wpose1.position.y += scale * 0.1
# print ("Pose 1: %s" % wpose1)
# waypoints.append(wpose1)

# wpose2.position.y -= scale * 0.2
# print ("Pose 2: %s" % wpose2)
# waypoints.append(wpose2)

# waypoints.append(wpose3)

# # waypoints, interpolation step, jump threshold
# (plan, fraction) = group.compute_cartesian_path(waypoints, 0.001, 0.0)
# print("Calculated fraction: %s" % fraction)
# group.execute(plan, wait=True)
