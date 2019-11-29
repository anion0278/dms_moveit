import time
import os

class MeasurementLogger(object):
    def __init__(self, tested_param_name, tested_param_value, measurements_full_dir_path):
        log_file_path = os.path.join(measurements_full_dir_path, tested_param_name, str(tested_param_value) + "_measurement.txt")
        # create dirs if dont exist
        if not os.path.exists(os.path.dirname(log_file_path)):
            os.makedirs(os.path.dirname(log_file_path))
        self.__log_file = open(log_file_path, "w+")

    def log_message(self, message):
        self.__log_file.write(message + "\r\n")
        print("Logged: " + message)

    def log_section(self, section_name):
        self.log_message("------- %s -------" % section_name)

    def log_param(self, param_name, param_value):
        self.log_message(param_name + ": %s" % param_value)

    def log_error(self, message):
        self.log_message("[ERROR]: %s" % message)

    def log_planning_and_execution_time(self, movement_name, measured_time):
        self.log_message(movement_name + " - planning: " +
                         str(measured_time[0]))
        self.log_message(movement_name + " - execution: " +
                         str(measured_time[1]))

    def close(self):
        self.__log_file.close()


class PostponedLogger(MeasurementLogger):
    def __init__(self, tested_param_name, tested_param_value, measurements_full_dir_path):
        super(PostponedLogger, self).__init__(tested_param_name, tested_param_value + "_short_successful_attempts", measurements_full_dir_path)
        self.postponed_messages = []
    
    def log_message(self, message):
        self.postponed_messages.append(message + "\r\n")
        print("Logged: " + message)

    def dispose_messages(self):
        self.postponed_messages = []

    def write_messages(self):
        for message in self.postponed_messages:
            super(PostponedLogger, self).log_message(message)