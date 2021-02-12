#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Pose
import numpy as np

import rosnode
import geometry_msgs.msg
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse


home_position = "Home"
group_name = "manipulator"
clear_octomap_service = "/clear_octomap"
state_validity_service = "/check_state_validity"

def get_joint_pose_from_deg(name, joint_angles_deg):
    return NamedJointPose(np.radians(np.array(joint_angles_deg)), name)

class NamedJointPose():
    def __init__(self, joint_angles_rad, name):
        self.pose = joint_angles_rad
        self.name = name

class NamedPointPose():
    def __init__(self, pose, name):
        if not isinstance(pose, Pose):
            raise AttributeError("Argument should be pose")
        self.pose = pose
        self.name = name


point_pose_A = NamedPointPose(
    Pose(Point(0.466, -0.0993, 0.8346),
         Quaternion(0.4892, 0.522, -0.4777, 0.5098)),
    "Point pose A")

point_pose_B = NamedPointPose(
    Pose(Point(0.1061, 0.4709, 0.8488),
         Quaternion(0.1039, 0.701, -0.0872, 0.6994)),
    "Point pose B")


joint_pose_A = NamedJointPose([1.12161887833653935,
                               -0.8200713084170896,
                               0.9261690554869411,
                               -1.6652095580852606,
                               -1.5501685915793058,
                               1.0543118993180052],
                              "Pose A")

joint_pose_B = NamedJointPose([2.582419230110138,
                               -0.8036786161659402,
                               0.8430760165715983,
                               -1.6316082500996831,
                               -1.5601357714877526,
                               1.3836915426267309],
                              "Pose B")


class RobotDriver: # TODO rename more appropriatelly?
    def __init__(self, total_speed = 1.0, total_acc = 1.0):
        rospy.wait_for_service(clear_octomap_service)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('python_moveit_commander', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.move_group.allow_replanning(False)
        self.move_group.set_num_planning_attempts(0)
        self.move_group.set_planner_id("RRTConnect")

        # this is needed in melodic(for some reason), otherwise the robot moves superslowly
        self.set_speed_params(total_speed, total_acc)
        self.__clear_octomap_service = rospy.ServiceProxy(clear_octomap_service, Empty)

    def set_speed_params(self, speed, acceleration):
        self.move_group.set_max_velocity_scaling_factor(speed)
        self.move_group.set_max_acceleration_scaling_factor(acceleration)

    def add_hmi_obj(self, pose, name, radius): #TODO into visualizer ? 
        self.scene.add_sphere(name, pose, radius)  

    def move_hmi_obj(self, pose, name, radius):
        self.scene.move_object(name, pose)

    def remove_hmi_obj(self, name):
        self.scene.remove_world_object(name)

    def cleanup(self):
        self.move_group.clear_path_constraints()
        self.move_group.clear_pose_targets()

    def clear_octomap(self):
        print("Octomap clearing...")
        self.__clear_octomap_service()

    def move_to_pose(self, target):
        '''Returns measured time - for planning and for execution'''
        self.set_target_pose(target)
        if not self.perform_planning(): return False
        success = self.execute_planned_sync()
        return success

    def stop_async_movement(self): 
        # '''Async movements only!''' 
        self.move_group.stop() 

    def get_pose(self):
        return self.move_group.get_current_joint_values()

    def set_target_pose(self, target):
        if (isinstance(target, str)):
            self.move_group.set_named_target(target)
        if (isinstance(target, NamedJointPose)):
            self.move_group.set_joint_value_target(target.pose)
        if (isinstance(target, NamedPointPose)):
            self.move_group.set_pose_target(target.pose)

    def get_motion_status(self, motion_feedback_msg):
        if motion_feedback_msg is None: return None
        # Feedback codes can be found at: http://docs.ros.org/en/fuerte/api/control_msgs/html/__FollowJointTrajectoryActionResult_8py_source.html
        move_status = motion_feedback_msg.status.status
        if move_status in [0, 1]: return None
        if move_status in [2, 3, 6]: return True
        if move_status in [4, 5, 7, 8, 9]: return False

    def perform_planning(self):
        plan = self.move_group.plan()

        max_acc = 0
        try:
            for point, i in zip(plan[1].joint_trajectory.points, range(len(plan[1].joint_trajectory.points))):
                point_acc_max = max(np.absolute(np.array(point.accelerations)))
                if len(point.accelerations) == 0:
                    point.accelerations = plan[1].joint_trajectory.points[i+1].accelerations
                if len(point.velocities) == 0:
                    point.velocities = plan[1].joint_trajectory.points[i+1].velocities
                # print("solved!!!")
                if point_acc_max > max_acc:
                    max_acc = point_acc_max
        except:
            pass
        # print("Motion - MAX ACC: %s" % max_acc)       
        success = plan[0] # moveit python API has changed
        if (not success):
            print("Could not plan the movement!")  # Plan: " + str(plan)
        return success

    def execute_planned_sync(self):
        success = self.move_group.go(wait=True)
        #print("New position: " + str(self.move_group.get_current_pose()))
        if (not success):
            print("Could not execute the movement!")
        return success

    def execute_planned_async(self):
        success = self.move_group.go(wait=False)
        return success

    def move_home(self):
        print("Moving to Home position")
        return self.move_to_pose(home_position)

    def __move_from_A_to_B(self):
        self.move_to_pose(joint_pose_A)
        self.move_to_pose(joint_pose_B)

    def execute_cyclic_movement(self, repetitions):
        self.move_home()
        print("Moving in cycles")
        for index in range(1, repetitions):
            print("Cycle %s in %s" % (index, repetitions - 1))
            self.__move_from_A_to_B()
        self.move_home()

if __name__ == "__main__":
    pass
    # robot_driver = RobotDriver()
    # robot_driver.execute_cyclic_movement(50)
