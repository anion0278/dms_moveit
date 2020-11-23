import csv
import numpy as np
import os
import re
import matplotlib.pyplot as plt
import json
import moveit_tester
from functools import reduce
import collections
import data_reader
from statistics import mean
from matplotlib.pyplot import cm
import matplotlib.lines as mlines
from difflib import SequenceMatcher

plt.rcdefaults()

current_script_path = os.path.dirname(__file__)
measurements_dir = os.path.join(current_script_path, "measurements")

margin = 0.005
x_labels_rotation = 70
bar_width = 0.8
chart_label_font = 7
figure_size = (10, 3.5) #(14, 4) # 10, 3.5
available_colors = ["#4e79a7", "#f28e2b", "#76b7b2",
                    "#e15759",   "#59a14f",  "#edc948", "#b07aa1", ]

def longestSubstringFinder(string1, string2):
    match = SequenceMatcher(None, string1, string2).find_longest_match(0, len(string1), 0, len(string2))
    return string1[match.a: match.a + match.size]

planning_min_max = []
execution_min_max = []

class ResultPlotter:
    def __init__(self, meas_reader):
        self.reader = meas_reader

    def show_bar_plot(self, bars_dict, title, y_label, y_limit=None):
        x_labels = []
        y_values = []
        for key, label in sorted(bars_dict.items()):
            x_labels.append(key)
            y_values.append(label)
        # x_labels = bars_dict.keys()
        x_pos = np.arange(len(x_labels))
        # y_values = bars_dict.values()

        fig = plt.figure(figsize=figure_size)
        # axes = plt.gca()
        # axes.yaxis.grid()
        plt.autoscale(enable=True, axis="x", tight=True)
        bar_chart = plt.bar(x_pos, y_values, align="center", width=bar_width)
        plt.xticks(x_pos, x_labels, fontsize=chart_label_font,
                   rotation=x_labels_rotation)
        plt.title(title)
        plt.ylabel(y_label)

        if y_limit != None:
            plt.ylim(y_limit)
        fig.tight_layout()
        plt.margins(0.01, 0)
        self.autolabel_bars(bar_chart)
        self.save_figure(title)
        self.show_figure(fig)

    def autolabel_bars(self, bar_chart):
        for bar in bar_chart:
            height = bar.get_height()
            y_pos = height/2 
            #if height > chart_label_font else height
            plt.annotate("{}".format(round(height, 2)),
                         xy=(bar.get_x() + bar.get_width() / 2,  y_pos),
                         xytext=(0, 0),
                         textcoords="offset points",
                         color="black",
                         fontsize=chart_label_font,
                         ha="center", va="bottom")

    def show_box_plot(self, statistics_dict, title, y_label, y_limit=None):
        all_data = []
        labels = []
        for key, label in sorted(statistics_dict.items()):
            labels.append(key)
            all_data.append(label)
        # all_data = statistics_dict.values()
        # labels = statistics_dict.keys()

        fig, axis = plt.subplots(nrows=1, ncols=1, figsize=figure_size)
        plot = axis.boxplot(all_data, vert=True,
                            patch_artist=True,  labels=labels)
        plt.xticks(fontsize=chart_label_font, rotation=x_labels_rotation)
        plt.ylabel(y_label)
        if y_limit != None:
            if (type(y_limit) is tuple):
                plt.ylim(bottom=y_limit[0], top=y_limit[1])

        fig.tight_layout()
        axis.set_title(title)
        axis.yaxis.grid(True, which='major')
        # plt.grid(color='black', which='major', axis='y', linestyle='solid')
        # axis.xaxis.grid(True, which='both')
        self.save_figure(title)
        self.show_figure(fig)

    def save_figure(self, file_title):
        plt.savefig(os.path.join(current_script_path, file_title + ".png"), dpi=600)

    def show_figure(self, figure):
        plt.draw()
        plt.waitforbuttonpress(0)
        plt.close(figure)

    def plot_all(self):
        self.show_relative_performances(moveit_tester.movements, self.reader, "Planner")
        self.show_durations_box_plots(moveit_tester.movements, self.reader)
        success_statistics = self.reader.get_success_statistics()
        self.show_bar_plot(success_statistics,
                           title="Success probability",
                           y_label="Success [%]")

    def show_durations_box_plots(self, movements, reader):
        for index in range(len(movements)):
            plan_statistics = reader.get_point_plan_statistics(index)
            self.show_box_plot(plan_statistics,
                               title="Planning time " + movements[index],
                               y_label="Time [s]",
                               y_limit=(0, 5))

            execution_statistics = reader.get_point_execution_statistics(index)
            y_limit = (3, 8) if index == 0 else None
            self.show_box_plot(execution_statistics,
                               title="Execution time " + movements[index],
                               y_label="Time [s]",
                               y_limit=y_limit)

    def show_relative_performances(self, movements, reader, exclude_name=""):
        relative_performances = self.calculate_relative_performance(
            movements, reader)
        # if exclude_name is not "":
        # relative_performances = collections.OrderedDict({k: v for k, v in relative_performances.items() if not exclude_name in k})
        self.filter_statistics_by_rule(
            relative_performances, lambda key, value: "obstacle" in key.lower())
        self.show_grouped_bar_plot(relative_performances,
                                   title="Relative performances - Obstacle 1 (Real robot)",
                                   y_label="Relative average perfomance [%]")
        self.show_relative_change_bar_plot(relative_performances)

    def calculate_relative_performance(self, movements, reader):
        relative_performances = collections.OrderedDict()
        for index in range(len(movements)):
            plan_durations = reader.get_point_plan_average(index)
            max_duration = max(plan_durations.values())
            min_duration = min(plan_durations.values())
            for key, value in sorted(plan_durations.items()):
                rel_value = moveit_tester.calculate_relative(
                    value, min_duration, max_duration)
                if not (key in relative_performances):
                    relative_performances[key] = []
                relative_performances[key].append(rel_value)

            # TODO refactoring - duplication
            execution_durations = reader.get_point_execution_average(index)
            max_exec_duration = max(execution_durations.values())
            min_exec_duration = min(execution_durations.values())
            for key, value in execution_durations.items():
                rel_value = moveit_tester.calculate_relative(
                    value, min_exec_duration, max_exec_duration)
                if not (key in relative_performances):
                    relative_performances[key] = []
                relative_performances[key].append(rel_value)

        success_probability = reader.get_success_statistics()
        for key, value in success_probability.items():
            if key in relative_performances:
                relative_performances[key].append(value)

        return relative_performances

    def filter_statistics_by_rule(self, statistics, exclude_rule):
        pass
        # for key, value in statistics.items():
        #     if exclude_rule(key, value):
        #         del statistics[key]

    def show_relative_change_bar_plot(self, relative_performance):
        relative_changes = collections.OrderedDict()
        keys = relative_performance.keys()
        values = relative_performance.values()
        for index in range(0, len(relative_performance), 2):
            common_substring = longestSubstringFinder(keys[index], keys[index+1])
            param_name = common_substring
            if "Planner" in common_substring:
                param_name = "Planner:"
            val_before = keys[index + 1].replace(common_substring, "").replace("0-","")
            val_new = keys[index].replace(common_substring, "").replace("0-","")
            new_key = param_name + "\n " + val_before  + " -> " + val_new
            relative_changes[new_key] = mean(values[index]) - mean(values[index + 1])
        y_limit = (-60, 20)
        self.show_bar_plot(relative_changes,
                           title="Relative performance change",
                           y_label="Relative change [%]",
                           y_limit=y_limit)

    def show_grouped_bar_plot(self, relative_performance, title, y_label):
        x_labels = relative_performance.keys()
        x_base = np.arange(len(x_labels))
        fig, axis = plt.subplots(figsize=(12, 4.5))

        members_labels = []
        for label in moveit_tester.movements:
            members_labels.append("Plan " + label)
            members_labels.append("Execute " + label)
        members_labels.append("Success")

        actions_durations = np.array(relative_performance.values())
        group_members_num = len(actions_durations[0])
        groups_num = len(actions_durations)
        group_width = bar_width
        bars = []
        for group_member_index in range(group_members_num):
            column = actions_durations[:, group_member_index]
            single_bar_width = group_width / group_members_num
            x_pos = x_base - group_width / 2 + single_bar_width * \
                (group_members_num - group_member_index)
            bar = axis.bar(x_pos, column, single_bar_width,
                           label=members_labels[group_member_index],
                           color=available_colors[group_member_index])
            bars.append(bar)

        avrg_lines_color = "blue"
        # average lines
        plt.margins(margin, 0)
        for group_index in range(groups_num):
            group_mean = mean(actions_durations[group_index])
            line_start = float(group_index) / groups_num + \
                margin * (groups_num - group_index) / groups_num
            line_end = float(group_index + 1) / groups_num - \
                margin * (group_index) / groups_num
            axis.axhline(y=group_mean,
                         xmin=line_start,
                         xmax=line_end,
                         linewidth=2, color=avrg_lines_color)

        average_legend = mlines.Line2D(
            [], [], color=avrg_lines_color, label="Total performance", linewidth=2)
        bars.append(average_legend)
        axis.legend(loc="center left", bbox_to_anchor=(1, 0.5),
                    fontsize=chart_label_font, handles=bars)
        axis.set_ylabel(y_label)
        axis.set_title(title)
        x_pos = x_base + group_width / group_members_num
        axis.set_xticks(x_pos)
        axis.set_xticklabels(x_labels, fontsize=chart_label_font,
                             rotation=x_labels_rotation)

        fig.tight_layout()
        plt.subplots_adjust(right=0.85) # 0.6 for Real , Sim comparison 0.8

        self.save_figure(title)
        self.show_figure(fig)


reader = data_reader.MeasurementsReader(measurements_dir)
reader.parse_all_measurements()

plotter = ResultPlotter(reader)
plotter.plot_all()
