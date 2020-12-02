import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import csv
import os
from matplotlib.patches import ConnectionPatch
from matplotlib.pyplot import text as pytext

import config as c

class meas():
       def __init__(self, name, y_label, color, max_val = 100, extra_y_ticks = None, y_ticks = None, is_num = True):
              self.name = name
              self.y_label = y_label
              self.timestamps = []
              self.vals = []
              self.prev_val = -1
              self.color = color
              self.max_val = max_val
              self.y_ticks = y_ticks
              self.extra_y_ticks = extra_y_ticks
              self.is_num = is_num

def process(row, meas):
       param_val = row[meas.name]
       try:
              if param_val != meas.prev_val:
                     meas.timestamps.append(float(row["time"]))
                     if meas.is_num:
                            meas.vals.append(float(param_val))
                     else:
                            meas.vals.append(param_val)
                     meas.prev_val = param_val
       except:
              pass

L2_inten = float(c.replan_intensity) / c.invalid_goal_intensity * 100

graphs = [
       meas("dist_right", "Min. distance\nHMI right [%]", "blue", 100), 
       meas("hmi_val_right", "Vibration val\nHMI right [%]", "orange", 100,
              extra_y_ticks=[L2_inten], y_ticks=[0, 50, "L2", "L1"]),
       meas("dist_left", "Min. distance\nHMI left [%]", "blue", 100),
       meas("hmi_val_left", "Vibration val\nHMI left [%]", "orange", 100,
              extra_y_ticks=[L2_inten], y_ticks=[0, 50, "L2", "L1"]),
       meas("dist_obj", "Min. distance\nto obstacles [%]", "blue", 100),
       meas("validity", "Validity of\ngoal pose", "green", 1, y_ticks=["F", "T"]),
       meas("replan", "Environment\nchanged", "green", 1, y_ticks=["F", "T"]),
       meas("goal_name", "Goal", "red", is_num=False) ]

current_script_path = os.path.dirname(os.path.realpath(__file__))

with open(os.path.join(current_script_path,'timeline.csv'), mode='r') as csv_file:
       csv_reader = csv.DictReader(csv_file)
       line_count = 0
       for row in csv_reader:
              for meas in graphs:
                     process(row, meas)
              line_count += 1

skip_meas = 1
next_goals = graphs[-1].timestamps[1:-1]

fig, ax = plt.subplots(len(graphs) - skip_meas, 1, sharex='col', sharey='row', figsize=(7, 9)) # w, h
for meas, index in zip(graphs, range(len(graphs) - skip_meas)):
       ax[index].tick_params(axis="y",labelsize=8)
       meas.timestamps.append(0)
       bin_width = np.diff(meas.timestamps)
       bar_plot = ax[index].bar(meas.timestamps[:-1], meas.vals,  color=meas.color,  width=bin_width,  align='edge')
       ax[index].set_ylabel(meas.y_label)
       if meas.max_val is not None:
              ax[index].set_ylim(0, meas.max_val)
       if meas.extra_y_ticks is not None:
              x = list(ax[index].get_yticks())
              ax[index].set_yticks(x + meas.extra_y_ticks)
       if meas.y_ticks is not None:
              if len(meas.y_ticks) == 2:
                     ax[index].locator_params(axis='y', nbins=len(meas.y_ticks)-1)
              ax[index].set_yticklabels(meas.y_ticks)
       ax[index].get_yaxis().set_label_coords(-0.08,0.5)
       for goal_change in next_goals:
              ax[index].axvline(goal_change, color='m', linestyle='--')
       #ax[index].yaxis.grid(True)
       ax[index].xaxis.grid(True)

for goal_change in next_goals:
       pytext(goal_change, 8.35, "Next goal", verticalalignment='center', size = 8, color = "m")

#plt.subplots_adjust(hspace=.0) # join
#plt.margins(x=0)

#plt.scatter(x, y)
# plt.grid(True)
plt.xlabel("Time [s]")
plt.show()