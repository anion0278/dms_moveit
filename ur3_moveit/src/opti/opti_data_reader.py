import csv
import numpy as np
import os
import re
import matplotlib.pyplot as plt
import json
from functools import reduce
import collections

import opti_moveit_tester


successful_attempts_name = "_short_successful_attempts_measurement.txt"
value_separator = ": "

class PointMeasurementResult:
    def __init__(self, point_name, plan_time, execution_time):
        self.point_name = point_name
        self.plan_time = plan_time
        self.execution_time = execution_time


class SingleMeasurementResult:
    def __init__(self, meas_number):
        self.measured_points = []
        self.number = meas_number

    def add_measured_point(self, point_result):
        self.measured_points.append(point_result)


class VariantMeasurementSeries:
    def __init__(self, param_value):
        self.measurements = []
        self.param_value = param_value

    def add_measurement(self, measurement_result):
        self.measurements.append(measurement_result)

    @property
    def success_rate(self):
        # FIXME should read value from file! because it could have changed
        return len(self.measurements) / float(opti_moveit_tester.test_repetitions)

    def get_average_execution_time_for_point(self, point_num):
        return self.__get_average_time(point_num, lambda x: x.execution_time)

    def get_point_plan_measurements(self, point_num):
        return self.__get_point_measurements(point_num, lambda x: x.plan_time)

    def get_point_execution_measurements(self, point_num):
        return self.__get_point_measurements(point_num, lambda x: x.execution_time)

    def __get_point_measurements(self, point_num, value_getter):
        point_measurements = []
        for meas in self.measurements:
            duration = value_getter(meas.measured_points[point_num])
            point_measurements.append(duration)
        return point_measurements

    def get_average_plan_time_for_point(self, point_num):
        return self.__get_average_time(point_num, lambda x: x.plan_time)

    def __get_average_time(self, point_num, value_getter):
        sum = 0
        if (len(self.measurements) == 0):
            return sum
        for measurement in self.measurements:
            sum += value_getter(measurement.measured_points[point_num])
        return sum / len(self.measurements)


class ParameterMeasurement:
    def __init__(self, param_name):
        self.variant_measurements = []
        self.param_name = param_name

    def add_variant_measurement(self, measurement):
        self.variant_measurements.append(measurement)


class MeasurementsReader:
    def __init__(self, measurements_dir):
        self.all_parameters_measured = []
        self.measurements_dir = measurements_dir

    def parse_all_measurements(self):
        for meas_dir in os.listdir(self.measurements_dir):
            param_measurement = ParameterMeasurement(meas_dir)
            current_meas_dir = os.path.join(self.measurements_dir, meas_dir)
            successful_meas_files = list(
                filter(lambda x: successful_attempts_name in x,  os.listdir(current_meas_dir)))
            for meas_file in successful_meas_files:
                variant_measurement = self.parse_measurement_variant(
                    current_meas_dir, meas_file)
                param_measurement.add_variant_measurement(variant_measurement)
            self.all_parameters_measured.append(param_measurement)

    def parse_measurement_variant(self, current_meas_dir, meas_file):
        file_contents = self.__get_file_contents(current_meas_dir, meas_file)
        cycles_data = self.__split_into_cycles(file_contents)
        variant_measurement = VariantMeasurementSeries(
            meas_file.replace(successful_attempts_name, ""))
        for cycle_index in range(len(cycles_data)):
            single_meas = SingleMeasurementResult(cycle_index)
            cycle_statements = self.__get_cycle_statements(cycles_data[cycle_index])
            for index in range(0, len(cycle_statements), 2):
                point_res = self.__parse_point_meas(cycle_statements, index)
                single_meas.add_measured_point(point_res)
            variant_measurement.add_measurement(single_meas)
        return variant_measurement

    def __get_cycle_statements(self, single_cycle):
        return list(filter(lambda x: not x.isspace( ) and "time" in x, single_cycle.split("\r\n")[1:]))

    def __parse_point_meas(self, cycle_statements, index):
        name_time_pattern = ".+ ([a-zA-Z])+: ([\d]+.[\d]+)"
        plan_match = re.match(
            name_time_pattern, cycle_statements[index])
        execution_match = re.match(
            name_time_pattern, cycle_statements[index+1])
        if (not plan_match or not execution_match):
            raise KeyError("Invalid measurement file")
        point_res = PointMeasurementResult(
            plan_match.group(1),
            float(plan_match.group(2)),
            float(execution_match.group(2)))
        return point_res

    def __split_into_cycles(self, file_contents):
        file_contents = re.sub(
            r"\r\n\r\n====== CYCLE [\d]+ END ======\r\n\r\n", "", file_contents)
        file_contents = re.sub(r"\r\n\r\n", "\r\n", file_contents)
        return re.split("====== CYCLE [\d]+ ======", file_contents)[1:]

    def __get_file_contents(self, current_meas_dir, meas_file):
        file = open(os.path.join(current_meas_dir, meas_file), mode="r")
        file_contents = file.read()
        return file_contents

    def get_point_plan_statistics(self, point_num):
        return self.__get_point_stat_dict(lambda variant: variant.get_point_plan_measurements(point_num), lambda samples: len(samples) > 0)

    def get_point_execution_statistics(self, point_num):
        return self.__get_point_stat_dict(lambda variant: variant.get_point_execution_measurements(point_num), lambda samples: len(samples) > 0)

    def get_point_plan_average(self, point_num):
        return self.__get_point_stat_dict(lambda variant: variant.get_average_plan_time_for_point(point_num), lambda average: average != 0)

    def get_point_execution_average(self, point_num):
        return self.__get_point_stat_dict(lambda variant: variant.get_average_execution_time_for_point(point_num), lambda average: average != 0)

    def get_success_statistics(self):
        return self.__get_point_stat_dict(lambda x: x.success_rate * 100, lambda average: average != 0)

    def __get_point_stat_dict(self, value_getter, validation = None):
        measuremetns_by_param_names = collections.OrderedDict()
        for param in self.all_parameters_measured:
            for param_variant in param.variant_measurements:
                name_base = param.param_name + value_separator + param_variant.param_value
                variant_meas = value_getter(param_variant)
                # if (not self.is_planning_fast(param_variant)): continue
                if ( validation == None or (validation != None and validation(variant_meas))):
                    measuremetns_by_param_names[name_base] = variant_meas
        return measuremetns_by_param_names

    def is_planning_fast(self, param_variant):
        for point_num in range(3):
            if (param_variant.get_average_plan_time_for_point(point_num) > 1):
                return False
        return True
