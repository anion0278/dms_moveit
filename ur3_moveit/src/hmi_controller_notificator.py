
import rospy

import config

obj_clearance_param = "/move_group/collision/min_clearance"

class TimedCommand():
    def __init__(self, intensity, time):
        self.time = time
        self.intensity = intensity


class VibroNotificator():
    def __init__(self, device_name):
        # self.__time_step_s = 0.05
        self.clearance_min = 0.15 
        self.hmi_status_param = "hmi_value"
        self.__min_vib = config.dist_intensity_min
        self.__max_vib = config.dist_intensity_max
        self.this_hmi_clearance_param = obj_clearance_param + "_" + config.get_side(device_name)
        self.second_hmi_clearance_param = config.get_second_hmi_name(self.this_hmi_clearance_param)

    # self.send_speed_command(self, speed):
    # !!!!!!!!!!!!!! super important
    #     if speed_comps[i] < config.vibr_min and speed_comps[i] > config.vibr_min / 2:
    #     val = config.vibr_min

    def calc_intensity(self):
        hmi_status_command = rospy.get_param(self.hmi_status_param)
        if hmi_status_command != 0:
            if hmi_status_command == config.status_invalid:
                return config.invalid_goal_intensity

            if hmi_status_command == config.status_replan:
                return TimedCommand(config.replan_intensity, 0.3)    
        else:
            this_hmi_clearance_level = rospy.get_param(self.this_hmi_clearance_param)
            second_hmi_clearance_level = rospy.get_param(self.second_hmi_clearance_param)
            objs_clearance_level = rospy.get_param(obj_clearance_param)

            # choose the smallest value if the smallest value is not the same as for second hmi
            if objs_clearance_level < this_hmi_clearance_level and second_hmi_clearance_level != objs_clearance_level:  
                this_hmi_clearance_level = objs_clearance_level
    
            if this_hmi_clearance_level < self.clearance_min:
                vibration_level = (self.__max_vib - self.__min_vib) * (self.clearance_min -
                    this_hmi_clearance_level) / self.clearance_min + self.__min_vib
                return vibration_level
            else:
                return 0

    def get_motor_speeds(self, speed_vector): # TODO decide here whether activate all motors simulateneously or use pattern activation
        speed_comps = [0,0,0,0,0,0]
        # components x, y, z, -x, -y, -z
        for i in range(len(speed_comps) / 2):
            if speed_vector[i] > 0: # positive
                speed_comps[i] = speed_vector[i]
            else: # negative number
                speed_comps[i + 3] = abs(speed_vector[i])

        #print("Device %s : %s" % (self.device_name, speed_comps))
        return speed_comps

    def init_params(self):
        self.__set_param(self.hmi_status_param, 0)
        self.__set_param(obj_clearance_param, self.clearance_min)
        self.__set_param(self.this_hmi_clearance_param, self.clearance_min)
        self.__set_param(self.second_hmi_clearance_param, self.clearance_min)

    def get_this_hmi_clearance(self):
        return self.__get_clearance(self.second_hmi_clearance_param)

    def get_second_hmi_clearance(self):
        return self.__get_clearance(self.this_hmi_clearance_param)

    def get_obj_clearance(self): # min distance from robot to unknown object
        return self.__get_clearance(obj_clearance_param)

    def __get_clearance(self, param_name):
        return rospy.get_param(param_name)

    def __set_param(self, param_name, value):
        rospy.set_param(param_name, value)


        # TODO VIBRATION should be proportional to the distance to the future trajectory AND !
        # AND proportional to the distance to the robot - if the user is in the path of trajectory,
        # then he still has time to react, no need to vibrate intesivelly. Requires changes in MoveIt!

        # Possible way to implement - counting how far from current position is the collision