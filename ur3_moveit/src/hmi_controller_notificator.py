import rospy
import ros_numpy

import config
import util_common as utils


class PromptNotification(): 
    def __init__(self, intensity, directed_vibration = True):
        self.intensity = intensity
        self.directed_vibration = directed_vibration

# for notificaitons that last for time period
class ProlongedNotification(PromptNotification): 
    def __init__(self, intensity, time_s, directed_vibration = True):
        self.time = time_s
        super().__init__(intensity, directed_vibration)

class VibroNotificator():
    def __init__(self, device_name, directed_vibration):
        self.directed_vibration_mode = directed_vibration
        self.reaction_dist_m = 0.15  # distance on which HMIs start to react (proportionally)
        self.__min_vib = config.dist_intensity_min
        self.__max_vib = config.dist_intensity_max
        self.hmi_clearance_param = config.clearance_param_name + "_" + config.get_side(device_name)

    def get_notification(self):
        task_status = self.get_task_status()
        if task_status == config.TaskStatus.REPLANNING:
            return ProlongedNotification(config.replan_intensity, 0.3, False)    
            
        if task_status == config.TaskStatus.INVALID_GOAL:
            return PromptNotification(config.invalid_goal_intensity, False)

        if task_status == config.TaskStatus.OK:
            this_hmi_clearance = self.get_hmi_clearance()
            obstacle_clearance = self.get_obstacle_clearance()
            # obstacle is closer than HMI
            # TODO decide if react on this event. Maybe the reaction dist should be lower?
            if obstacle_clearance < this_hmi_clearance:   
                this_hmi_clearance = obstacle_clearance
            if this_hmi_clearance < self.reaction_dist_m:
                return PromptNotification(self.__get_proportional_vibration(this_hmi_clearance))
        return PromptNotification(0, directed_vibration=False)

    def __get_proportional_vibration(self, clearance):
        return  (self.__max_vib - self.__min_vib) * (self.reaction_dist_m - clearance) / self.reaction_dist_m + self.__min_vib

    def get_motor_speeds(self, notification, ros_vec): 
        speed_vector = notification.intensity * ros_numpy.numpify(ros_vec)
        if self.directed_vibration_mode and notification.directed_vibration:
            speed_comps = self.__directed_vibration(speed_vector)
        else:
            speed_comps = self.__simulateneous_vibration(notification.intensity)
        #print("Device %s : %s" % (self.device_name, speed_comps))
        return speed_comps

    #     if speed_comps[i] < config.vibr_min and speed_comps[i] > config.vibr_min / 2:
    #     val = config.vibr_min

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
        return speed_comps

    def init_params(self):
        utils.set_param(config.task_status_param_name, 0)
        utils.set_param(config.clearance_param_name, self.reaction_dist_m)
        utils.set_param(self.hmi_clearance_param, self.reaction_dist_m)

    def get_hmi_clearance(self):
        return utils.get_param(self.hmi_clearance_param)

    def get_obstacle_clearance(self): # min distance from robot to unknown object
        return utils.get_param(config.clearance_param_name)

    def get_task_status(self): 
        return utils.get_param(config.task_status_param_name)

    # TODO Vibration may be proportional to the distance to the future trajectory 
    # AND proportional to the distance to the robot - if the user is in the path of trajectory,
    # then he still has time to react, no need to vibrate intesivelly. Requires changes in MoveIt!
    # Possible way to implement - counting how far from current position is the collision