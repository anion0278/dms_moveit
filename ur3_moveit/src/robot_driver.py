#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def move_to_joint_pose(joint_values):
    group.set_joint_value_target(joint_values)
    group.plan()
    group.go(wait=True)


def move_home():
    print("Moving to Home position")
    group.set_named_target(home_position)
    group.plan()
    group.go(wait=True)


def move_from_A_to_B():
    start_motion_joint_pose = [-0.45161887833653935,
                               -0.8200713084170896,
                               0.9261690554869411,
                               -1.6652095580852606,
                               -1.5501685915793058,
                               1.0543118993180052]

    move_to_joint_pose(start_motion_joint_pose)

    end_motion_joint_pose = [1.112419230110138,
                             -0.8036786161659402,
                             0.8430760165715983,
                             -1.6316082500996831,
                             -1.5601357714877526,
                             1.3836915426267309]

    move_to_joint_pose(end_motion_joint_pose)


home_position = "Home"
group_name = "manipulator"

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_python_driver', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander(group_name)

# group.set_goal_orientation_tolerance(10)
# group.set_goal_position_tolerance(10)

print("Robot current state")
print robot.get_current_state()
print group.get_current_pose()

move_home()

print("Moving in cycles")
for index in range(0, 4):
    move_from_A_to_B()

move_home()


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
