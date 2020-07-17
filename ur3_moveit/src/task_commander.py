#!/usr/bin/env python

import robot_driver as r
import rospy
import sys
import config

hmi_status_param = "hmi_value"
env_changed_param = "/replan"
goal_name_param = "/goal_name"

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
        if wait_for_hmi == "true":
            self.wait_for_hmi()

        self.robot_driver = robot_driver
        self.num_attempts = num_attempts
        self.status_ok()
        self.set_goal_validity(True)
        self.set_env_change(False)
        self.set_goal_name("none")
        pass

    def wait_for_hmi(self):
        print("HMI: Waiting for RIGHT HMI")
        rospy.wait_for_service("hmi_glove_right_service")
        print("HMI: Waiting for LEFT HMI")
        rospy.wait_for_service("hmi_glove_left_service")

    def move_to(self, pose, retreat_on_fail=True):
        if isinstance(pose, str):
            rospy.set_param(goal_name_param, pose)
        if isinstance(pose, r.NamedJointPose):
            rospy.set_param(goal_name_param, pose.name)
        
        retreat_pose = r.NamedJointPose(self.robot_driver.get_pose(),
                                            "Retreat pose")
        plan_attempts = 0
        move_attempts = 0
        success = False
        while not success and (self.num_attempts == 0 or
                               (plan_attempts < self.num_attempts
                                and move_attempts < self.num_attempts)):
            
            self.robot_driver.set_target_pose(pose)
            plan_attempts += 1
            if not self.replan():
                self.set_goal_validity(False)
                self.status_unable_to_plan()
                continue

            self.set_goal_validity(True)
            rospy.set_param(hmi_status_param, 0)
            move_attempts += 1
            if not self.robot_driver.execute_planned_sync():
                self.status_movement_interrupted()
            else:
                success = True

        if not success and retreat_on_fail:
            self.retreat_to_previous_pose(retreat_pose)

        return success

    def replan(self):
        #set_env_change(True)
        result = self.robot_driver.perform_planning()
        #set_env_change(False)
        return result

    def retreat_to_previous_pose(self, retreat_pose):
        self.robot_driver.set_target_pose(retreat_pose)
        if not self.replan():
            # Retreating and notifying that there is no possible plan
            self.status_unable_to_plan()
        print("Retreat...")
        if self.robot_driver.execute_planned_sync():
            print("Sucessfull retreat")
        else:
            print("Unsucessfull retreat !")

    def set_goal_validity(self, validity):
        rospy.set_param("/goal_validity", validity)

    def set_goal_name(self, name):
        rospy.set_param(goal_name_param, name)

    def set_env_change(self, state):
        rospy.set_param(env_changed_param, state)

    def status_unable_to_plan(self):
        rospy.set_param(hmi_status_param, config.status_invalid)

    def status_movement_interrupted(self):
        rospy.set_param("/replan", True)
        rospy.set_param("/env_change_impulse", True)
        rospy.set_param(hmi_status_param, config.status_replan)
        #rospy.sleep(duration=0.3)
        rospy.set_param("/replan", False)

    def status_ok(self):
        rospy.set_param(hmi_status_param, 0)

    def demo(self):
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
                print(ex.message)
                print("STOPPED")
                pass


if __name__ == "__main__":
    driver = r.RobotDriver(total_speed=0.2, total_acc=0.9)
    args = rospy.myargv(argv=sys.argv)
    wait_for_hmi = args[1] if len(args) > 1 else False
    commander = Commander(driver, wait_for_hmi, num_attempts=0) # 0 attempts -> infinite
    commander.demo()