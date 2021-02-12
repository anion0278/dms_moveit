#!/usr/bin/env python

import rospy
import sys
from ur_msgs.srv import *
from icecream import ic
import curses, time
import setproctitle
setproctitle.setproctitle("DMS Task Commander")

import robot_driver as r
import util_keyboard as keyboard
import config
from config import TaskStatus
import util_common as utils
import hmi_controller_starter as starter
from moveit_msgs.msg import MoveGroupActionFeedback

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

def set_real_robot_speed_slider(slider_pos):
    if slider_pos > 1 or slider_pos < 0:
        raise AttributeError("Slider position should be in range 0..1")
    speed_slider_service = rospy.ServiceProxy("/ur_hardware_interface/set_speed_slider", SetSpeedSliderFractionRequest)
    msg = SetSpeedSliderFractionRequest()
    msg.speed_slider_fraction = slider_pos
    speed_slider_service(msg)

class Commander():
    def __init__(self, robot_driver, wait_for_hmi, num_attempts):
        self.robot_driver = robot_driver
        self.robot_driver.clear_octomap()
        if wait_for_hmi:
            self.__wait_for_hmi()
        self.robot_driver.clear_octomap()
        self.num_attempts = num_attempts
        self.plan_attempts_before_octo_clearing = 8
        self.current_movement_status = None
        self.async_movement_execution_monitor = rospy.Subscriber("/move_group/feedback", MoveGroupActionFeedback, callback=self.on_movement_executed)
        self.__init_status()

    def on_movement_executed(self, feedback_msg): # TODO should be in RobotDriver
        self.current_movement_status = feedback_msg

    def __init_status(self):
        self.__set_status(TaskStatus.OK)
        self.__set_debug_goal_validity(True)
        self.__set_debug_plan_interrupted(False)
        self.__set_debug_goal_name("None")


    def move_to(self, pose, retreat_on_fail=True, enable_preempting = False):
        retreat_pose = self.__get_retreat_pose()
        self.__set_debug_goal_name(pose)
        plan_attempts = 0
        move_attempts = 0
        success = False
        while not success and (self.num_attempts == 0 or
                               (plan_attempts < self.num_attempts
                                and move_attempts < self.num_attempts)):
            
            self.robot_driver.set_target_pose(pose)
            if plan_attempts > self.plan_attempts_before_octo_clearing:
                print("Trying to clean out octomap noise:")
                self.robot_driver.clear_octomap() # solves occasional octomap noise-probles 
                time.sleep(0.2) # give the octomap enough time to regenerate

            plan_attempts += 1
            if not self.replan():
                self.__set_debug_goal_validity(False)  # TODO refactoring
                self.__set_status_unable_to_plan()
                continue
            plan_attempts = 0 # counting only sequential unsuccesfull attempts

            self.__set_debug_goal_validity(True) # TODO refactoring, DRY
            self.__set_status(TaskStatus.OK)
            move_attempts += 1

            if not enable_preempting:
                if not self.robot_driver.execute_planned_sync():
                    self.__set_status_interrupted()
                else:
                    success = True
            else:
                success = self.execute_planned_async()

        if not success and retreat_on_fail:
            self.retreat_to_previous_pose(retreat_pose)

        return success

    def execute_planned_async(self):
        ''' Enables preemption '''
        success = False
        self.current_movement_status = None
        move_status = None
        self.robot_driver.execute_planned_async()
        print("< Press ESC to stop movement execution >") 
        while move_status is None: # TODO Into util_keyboard
            pressed_key_code = curses.wrapper(keyboard.key_hook)
            if pressed_key_code == 27:
                success = True
                print("ESC - Motion stopped - Keyboard interrupt!")
                self.robot_driver.stop_async_movement()
                break
            time.sleep(0.05)
            move_status = self.robot_driver.get_motion_status(self.current_movement_status)
        if not success:
            success = move_status
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
            print("Unsucessfull retreat!")

    def __wait_for_hmi(self):
        starter.wait_for_hmi(config.hmi_left)
        starter.wait_for_hmi(config.hmi_right)

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

    def demo_task_sequence(self, task_poses, repetitions = 1000, move_to_home = True, enable_preemtping = False):
        try:
            if move_to_home:
                self.move_to(r.home_position, enable_preemtping)
            for i in range(0, repetitions):
                for pose in task_poses:
                    if rospy.is_shutdown(): sys.exit() # it will take time to exit
                    if not self.move_to(pose, enable_preemtping):
                        print("The movement could not been executed!")
                        raise MovementFailed
            if move_to_home:
                self.move_to(r.home_position, enable_preemtping)
        except Exception as ex:
            print("TASK COMMANDER STOPPED: %s" % ex.message)


def get_task_commander():
    args = rospy.myargv(argv=sys.argv)
    wait_for_hmi = args[1] == "true" if len(args) > 1 else False

    if len(args) > 2 and args[2] == "sim":
        print("Task Commander: Simulation mode")
    else: 
        print("Task Commander: Real robot mode")
        # set_real_robot_speed_slider(0.5)

    robot_speed = 0.9
    driver = r.RobotDriver(total_speed=robot_speed, total_acc=robot_speed)
    return Commander(driver, wait_for_hmi, num_attempts=0) # 0 attempts -> infinite


if __name__ == "__main__":
    commander = get_task_commander()
    commander.demo_task_sequence([r.joint_pose_A, r.joint_pose_B, r.joint_pose_A])

    # task_poses_l = [
    #     r.get_joint_pose_from_deg("Left-1",[-28,-79,97,-109,-89,0]),
    #     r.get_joint_pose_from_deg("Left-2",[10,-58,36,25,11,0]),
    #     r.get_joint_pose_from_deg("Left-3",[21,-19,23,-5,23,0]),
    #     r.get_joint_pose_from_deg("Left-1",[-28,-79,97,-109,-89,0]),
    #     r.get_joint_pose_from_deg("Right-1",[165,-79,97,-109,-89,0]),
    #     # r.get_joint_pose_from_deg("Right-2",[123,-53,24,28,126,0]),
    #     r.get_joint_pose_from_deg("Right-3",[130,-35,24,-81,-89,0]),
    #     r.get_joint_pose_from_deg("Right-1",[165,-79,97,-109,-89,0]),
    # ]


    # task_poses_s = [
    #     r.get_joint_pose_from_deg("Left-1",[-28,-79,97,-109,-89,0]),
    #     r.get_joint_pose_from_deg("Right-1",[165,-79,97,-109,-89,0]),
    # ]

