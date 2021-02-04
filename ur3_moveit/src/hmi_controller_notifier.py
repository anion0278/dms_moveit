import rospy
import ros_numpy

import config
from config import TaskStatus
import util_common as utils


class PromptNotification(): 
    def __init__(self, intensity, directed_vibration = True):
        self.intensity = intensity
        self.directed_vibration = directed_vibration

# for notificaitons that last for time period
class ProlongedNotification(): 
    def __init__(self, intensity, time_s, directed_vibration = True):
        self.time = time_s
        # its way better than Python's inheritance
        self.intensity = intensity 
        self.directed_vibration = directed_vibration


class VibroNotifier():
    def __init__(self, device_name, directed_vibration):
        self.directed_vibration_mode = directed_vibration
        self.reaction_dist_m = 0.20  # distance on which HMIs start to react (proportionally)
        self.__min_vib = config.dist_intensity_min
        self.__max_vib = config.dist_intensity_max
        self.hmi_clearance_param = config.clearance_param + "_" + device_name

    def get_notification(self):
        task_status = self.get_task_status()
        #print("Task status: %s" % task_status)
        if task_status == TaskStatus.INTERRUPTED:
            return ProlongedNotification(config.replan_intensity, 0.3, False)    
            
        if task_status == TaskStatus.INVALID_GOAL:
            return PromptNotification(config.invalid_goal_intensity, False)

        if task_status == TaskStatus.OK:
            clearance = self.__get_clearance()
            print("Clearance [m]: %s" % clearance)
            pv = self.__get_proportional_vibration(clearance)
            print("Vibration: %s" % pv)
            return PromptNotification(pv)

        raise AttributeError("Unrecognized task status")

    def __get_clearance(self):
        return self.get_hmi_clearance()
        # If obstacle is closer than HMI
        # TODO decide whether we should react on this event. Maybe the reaction dist should be lower?
        # obstacle_clearance = self.get_obstacle_clearance()
        # if obstacle_clearance < hmi_clearance:   
        #     hmi_clearance = obstacle_clearance

    def __get_proportional_vibration(self, clearance):
        if clearance < self.reaction_dist_m:
            return  (self.__max_vib - self.__min_vib) * (self.reaction_dist_m - clearance) / self.reaction_dist_m + self.__min_vib
        return 0

    def get_motor_speeds(self, notification, ros_vec): 
        speed_vector = notification.intensity * ros_numpy.numpify(ros_vec)
        if self.directed_vibration_mode and notification.directed_vibration:
            speed_comps = self.__directed_vibration(speed_vector)
        else:
            speed_comps = self.__simulateneous_vibration(notification.intensity)
        #print("Device %s : %s" % (self.device_name, speed_comps))
        return speed_comps

    def __simulateneous_vibration(self, s):
        return [s,s,s,s,s,s]

    def __directed_vibration(self, speed_vector):
        speed_comps = [0,0,0,0,0,0] # components x, y, z, -x, -y, -z
        # vector into speed components
        for i in range(len(speed_comps) / 2):
            if speed_vector[i] > 0: 
                speed_comps[i] = speed_vector[i]
            else: 
                speed_comps[i + 3] = abs(speed_vector[i])

        for i in range(len(speed_comps) / 2):   
            if speed_comps[i] < config.vibr_min and speed_comps[i] >= config.vibr_min / 2:
                speed_comps[i] = config.vibr_min

        return speed_comps

    def init_params(self):
        utils.set_param(config.task_status_param, 0)
        utils.set_param(config.clearance_param, self.reaction_dist_m)
        utils.set_param(self.hmi_clearance_param, self.reaction_dist_m)

    def get_hmi_clearance(self):
        return utils.get_param(self.hmi_clearance_param)

    def get_obstacle_clearance(self): # min distance from robot to unknown object
        return utils.get_param(config.clearance_param)

    def get_task_status(self): 
        return TaskStatus(utils.get_param(config.task_status_param))

    # TODO Vibration may be proportional to the distance to the future trajectory 
    # AND proportional to the distance to the robot - if the user is in the path of trajectory,
    # then he still has time to react, no need to vibrate intesivelly. Requires changes in MoveIt!
    # Possible way to implement - counting how far from current position is the collision