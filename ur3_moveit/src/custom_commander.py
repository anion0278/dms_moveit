import robot_driver as drive
import rospy

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
    # if num attempts < 0 - infinite attempts
    def __init__(self, num_attempts=-1):
        self.robot_driver = drive.RobotDriver()
        self.num_attempts = num_attempts
        rospy.set_param('hmi_value', 0)

    def move_to(self, pose, retreat_on_fail=True):
        retreat_pose = drive.NamedJointPose(self.robot_driver.get_pose(), "Retreat pose")
        plan_attempts = 0
        move_attempts = 0
        success = False
        while not success and (self.num_attempts < 0 or (plan_attempts < self.num_attempts and move_attempts < self.num_attempts)):
            self.robot_driver.set_target_pose(pose)
            plan_attempts += 1
            if not self.robot_driver.perform_planning():
                self.send_unable_to_plan()
                continue
            
            rospy.set_param('hmi_value', 0)
            move_attempts += 1
            if not self.robot_driver.execute_planned_sync():
                self.send_movement_interrupted()
            else:
                success = True

        if not success and retreat_on_fail:
            # Retreating 
            self.robot_driver.set_target_pose(pose)
            if not self.robot_driver.perform_planning():
                # Retreating and notifying that there is no possible plan
                self.send_unable_to_plan()
            print("Retreat...")
            if self.robot_driver.move_to_pose(retreat_pose):
                print("Sucessfull")
            else: 
                print("Unsucessfull !")
                pass
        return success

    def send_unable_to_plan(self):
        rospy.set_param('hmi_value', 120)

    def send_movement_interrupted(self):
        rospy.set_param('hmi_value', 80)
        rospy.sleep(duration=0.5)

if __name__ == "__main__":
    commander = Commander(num_attempts=4)

    try:
        commander.move_to(drive.home_position)
        list_poses = [drive.joint_pose_A, drive.joint_pose_B, drive.joint_pose_A]
        for i in range(0, 1000):
            for pose in list_poses:
                if commander.move_to(pose):
                    print("The movement could not been executed!")
                    raise MovementFailed
            
        commander.move_to(drive.home_position)
    except Exception as ex:
        print(ex.message)
        print("STOPPED")
        pass