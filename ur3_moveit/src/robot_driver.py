#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from math import pi
import time
import os
import ros_process

from std_msgs.msg import String
from std_srvs.srv import Empty
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import timeout
import rosnode

from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse

home_position = "Home"
group_name = "manipulator"
clear_octomap_service = "/clear_octomap"
state_validity_service = "/check_state_validity"


class NamedJointPose():
    def __init__(self, joint_angles, name):
        self.pose = joint_angles
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


joint_pose_A = NamedJointPose([-0.45161887833653935,
                               -0.8200713084170896,
                               0.9261690554869411,
                               -1.6652095580852606,
                               -1.5501685915793058,
                               1.0543118993180052],
                              "Joint pose A")

joint_pose_B = NamedJointPose([1.012419230110138,
                               -0.8036786161659402,
                               0.8430760165715983,
                               -1.6316082500996831,
                               -1.5601357714877526,
                               1.3836915426267309],
                              "Joint pose B")


def restart_moveit_node():
    success = False
    while success is False:
        try:
            print("Killing move group node")
            rosnode.kill_nodes(["/move_group"])
            ros_process.kill_process_by_name("move_group")
            rosnode.rosnode_cleanup(override=True)
            time.sleep(1)
            rospy.wait_for_service("/move_group/load_map", timeout=5)
            rospy.wait_for_service("/move_group/trajectory_execution/set_parameters", timeout=1)
            rospy.wait_for_service("/check_state_validity", timeout=1)
            rospy.wait_for_service(clear_octomap_service, timeout=5)
            service = rospy.ServiceProxy(clear_octomap_service, Empty)
            timeout.action_with_timeout(service, 5)
            success = True
        except Exception as ex:
            success = False


def set_dynamic_params(params):
    rospy.set_param("/move_group/octomap_resolution", float(params[0]))
    rospy.set_param("/point_subsample", int(params[1]))
    rospy.set_param("/move_group/manipulator/longest_valid_segment_fraction", float(params[2]))


class RobotDriver:
    def __init__(self):
        # self._base_pub = rospy.Publisher('/cartpole_v0/foot_joint_velocity_controller/command', Float64, queue_size=1)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_python_driver', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        rospy.wait_for_service(clear_octomap_service)
        self.__clear_octomap_service = rospy.ServiceProxy(clear_octomap_service, Empty)
        self.clear_octomap()

    def cleanup(self):
        self.move_group.clear_path_constraints()
        self.move_group.clear_pose_targets()

    def clear_octomap(self):
        print("Octomap clearing...")
        self.__clear_octomap_service()

    def __del__(self):
        self.cleanup()
        del self.robot
        del self.scene
        del self.move_group
        os.system("rosnode cleanup")

    # returns measured time - for planning and for execution

    def move_to_pose(self, target):
        self.set_target_pose(target)
        success = self.perform_planning()
        success = self.execute_planned_sync() and success
        return success

    def set_target_pose(self, target):
        if (isinstance(target, str)):
            self.move_group.set_named_target(target)

        if (isinstance(target, NamedJointPose)):
            self.move_group.set_joint_value_target(target.pose)

        if (isinstance(target, NamedPointPose)):
            self.move_group.set_pose_target(target.pose)

    def perform_planning(self):
        plan = self.move_group.plan()
        success = plan[0]
        if (not success):
            print("Could not plan the movement!")  # Plan: " + str(plan)
        return success

    def execute_planned_sync(self):
        success = self.move_group.go(wait=True)
        #print("New position: " + str(self.move_group.get_current_pose()))
        if (not success):
            print("Could not execute the movement!")
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


# def move_joints(self, joints_array):
#     joint_value = Float64()
#     joint_value.data = joints_array[0]
#     rospy.logdebug("Single Base JointsPos>>"+str(joint_value))
#     self._base_pub.publish(joint_value)

# def reset_controllers(self):
#     """
#     We turn on and off the given controllers
#     :param controllers_reset: ["name_controler_1", "name_controller2",...,"name_controller_n"]
#     :return:
#     """
#     reset_result = False

#     result_off_ok = self.switch_controllers(controllers_on = [],
#                             controllers_off = self.controllers_list)

#     rospy.logdebug("Deactivated Controlers")

#     if result_off_ok:
#         rospy.logdebug("Activating Controlers")
#         result_on_ok = self.switch_controllers(controllers_on=self.controllers_list,
#                                                 controllers_off=[])
#         if result_on_ok:
#             rospy.logdebug("Controllers Reseted==>"+str(self.controllers_list))
#             reset_result = True
#         else:
#             rospy.logdebug("result_on_ok==>" + str(result_on_ok))
#     else:
#         rospy.logdebug("result_off_ok==>" + str(result_off_ok))

#     return reset_result

if __name__ == "__main__":


    # rospy.wait_for_service('/controller_manager/switch_controller')

    # switch_controller = rospy.ServiceProxy(
    #                    '/controller_manager/switch_controller', SwitchController)


    # switch_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    # switch_request_object = SwitchControllerRequest()
    # switch_request_object.start_controllers = ['arm_controller', 'joint_state_controller']
    # switch_request_object.stop_controllers =  []#['arm_controller', 'joint_state_controller', 'joint_group_position_controller']
    # switch_request_object.strictness = 1

    # switch_result = switch_service(switch_request_object)
########################
    # msg = SwitchController()
    # msg.start_controllers = ['arm_controller']
    # msg.stop_controllers = []
    # msg.strictness = 1
    # msg = SwitchController(start_controllers = [], stop_controllers=['arm_controller', 'joint_state_controller'], strictness = 1)
    # ret = switch_controller([], ['joint_state_controller'], 1)
############################
    # ros_process.start_roscore()
    # octomap_res = 0.02
    # point_subs = 1
    # segment = 0.005
    
    # planner = "BiTRRT"
    # set_dynamic_params([octomap_res, point_subs, segment])
    # gazebo_sim = ros_process.Environment(
    #     is_obstacle_present=True,
    #     is_workspace_limited=True,
    #     is_simulated=True
    #     )
    # gazebo_sim.start()
    robot_driver = RobotDriver()

    # robot_driver.move_group.forget_joint_values("manipulator")
    robot_driver.execute_cyclic_movement(10)

    # ros_process.close_roscore()
