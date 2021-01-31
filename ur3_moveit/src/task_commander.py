#!/usr/bin/env python

import rospy
import sys

import robot_driver as r
import config
from config import TaskStatus
import util_common as utils


class MovementFailed(Exception):
    def __init__(self, *args):
        if args:
            self.message = args[0]
        else:
            self.message = None

    def __str__(self):
        if self.message:
            return 'Movement failed:, {0} '.format(self.message)
        else:
            return 'Movement failed!'


class Commander():
    def __init__(self, robot_driver, wait_for_hmi, num_attempts):
        if wait_for_hmi:
            self.__wait_for_hmi()
        self.robot_driver = robot_driver
        self.num_attempts = num_attempts
        self.__init_status()

    def __init_status(self):
        self.__set_status(TaskStatus.OK)
        self.__set_debug_goal_validity(True)
        self.__set_debug_plan_interrupted(False)
        self.__set_debug_goal_name("None")

    def demo_task(self):
        while True:
            try:
                commander.move_to(r.home_position)
                list_poses = [r.joint_pose_A, r.joint_pose_B, r.joint_pose_A]
                for i in range(0, 1000):
                    for pose in list_poses:
                        if not commander.move_to(pose):
                            print("The movement could not been executed!")
                            raise MovementFailed
                commander.move_to(r.home_position)

            except Exception as ex:
                print("TASK COMMANDER STOPPED: %s" % ex.message)
                pass

    def move_to(self, pose, retreat_on_fail=True):
        retreat_pose = self.__get_retreat_pose()
        self.__set_debug_goal_name(pose)
        plan_attempts = 0
        move_attempts = 0
        success = False
        while not success and (self.num_attempts == 0 or
                               (plan_attempts < self.num_attempts
                                and move_attempts < self.num_attempts)):
            
            self.robot_driver.set_target_pose(pose)
            plan_attempts += 1
            if not self.replan():
                self.__set_debug_goal_validity(False)
                self.__set_status_unable_to_plan()
                continue

            self.__set_debug_goal_validity(True)
            self.__set_status(TaskStatus.OK)
            move_attempts += 1
            if not self.robot_driver.execute_planned_sync():
                self.__set_status_interrupted()
            else:
                success = True

        if not success and retreat_on_fail:
            self.retreat_to_previous_pose(retreat_pose)

        return success

    def __get_retreat_pose(self):
        return r.NamedJointPose(self.robot_driver.get_pose(),"Retreat pose")

    def replan(self):
        #self.__set_debug_plan_interrupted(True) # Depends on whether usual Planning should treated as a change of plan
        result = self.robot_driver.perform_planning()
        #self.__set_debug_plan_interrupted(False)
        return result

    def retreat_to_previous_pose(self, retreat_pose):
        self.robot_driver.set_target_pose(retreat_pose)
        if not self.replan():
            # Retreating and notifying that there is no possible plan
            self.__set_status_unable_to_plan()
        print("Retreat...")
        if self.robot_driver.execute_planned_sync():
            print("Sucessfull retreat")
        else:
            print("Unsucessfull retreat !")

    def __wait_for_hmi(self):
        print("Task Commander: Waiting for Left HMI")
        rospy.wait_for_service(config.hmi_left + config.calibr_service)
        print("Task Commander: Waiting for Right HMI")
        rospy.wait_for_service(config.hmi_right + config.calibr_service)

    def __set_debug_goal_name(self, pose):
        if isinstance(pose, str):
            utils.set_param(config.goal_name_param, pose)
        if isinstance(pose, r.NamedJointPose):
            utils.set_param(config.goal_name_param, pose.name)

    def __set_status(self, status_enum_val):
        utils.set_param(config.task_status_param, status_enum_val.value)

    def __set_debug_goal_validity(self, validity):
        utils.set_param(config.goal_validity_param, validity)

    def __set_debug_plan_interrupted(self, state):
        utils.set_param(config.plan_interrupted_param, state)

    def __set_status_unable_to_plan(self):
        self.__set_status(TaskStatus.INVALID_GOAL)

    def __set_status_interrupted(self):
        self.__set_debug_plan_interrupted(True)
        utils.set_param(config.replan_impulse_param, True) # to make sure that event was recorded
        self.__set_status(TaskStatus.INTERRUPTED)
        self.__set_debug_plan_interrupted(False)


if __name__ == "__main__":
    import time
    time.sleep(3) # othewise Moveit does not always start

    args = rospy.myargv(argv=sys.argv)
    wait_for_hmi = args[1] == "true" if len(args) > 1 else False
    robot_speed = 0.001 if len(args) > 2 and args[2] == "sim" else 0.7
    driver = r.RobotDriver(total_speed=robot_speed, total_acc=0.3)
    commander = Commander(driver, wait_for_hmi, num_attempts=0) # 0 attempts -> infinite
    commander.demo_task()
