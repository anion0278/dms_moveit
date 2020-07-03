import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import csv
import os
import config

#https://stackoverflow.com/questions/57164333/bar-chart-with-different-widths-and-colors-in-matplotlib-bar-mekko-chart


class meas():
       def __init__(self, name, y_label, color, max_val = 100):
              self.name = name
              self.y_label = y_label
              self.timestamps = [0]
              self.vals = []
              self.prev_val = -1
              self.color = color
              self.max_val = max_val

def process(row, meas):
       param_val = row[meas.name]
       try:
              if param_val != meas.prev_val:
                     meas.vals.append(float(param_val))
                     meas.prev_val = param_val
                     meas.timestamps.append(float(row["time"]))
       except:
              pass


graphs = [
       meas("dist_right", "Min. distance \n HMI right [%]", "blue", None),
       meas("hmi_val_right", "Vibration val \n HMI right [%]", "orange"),
       meas("dist_left", "Min. distance \n HMI left [%]", "blue", None),
       meas("hmi_val_left", "Vibration val \n HMI left [%]", "orange"),
       meas("dist_obj", "Min. distance \n to unknown obstacles [%]", "blue", None)]

current_script_path = os.path.dirname(os.path.realpath(__file__))

with open(os.path.join(current_script_path,'timeline.csv'), mode='r') as csv_file:
       csv_reader = csv.DictReader(csv_file)
       line_count = 0
       for row in csv_reader:
              for meas in graphs:
                     process(row, meas)
              line_count += 1

fig, ax = plt.subplots(len(graphs), 1, sharex='col', sharey='row', figsize=(7, 8)) # w, h
for meas, index in zip(graphs, range(len(graphs))):
       bin_width = np.diff(meas.timestamps)
       bar_plot = ax[index].bar(meas.timestamps[:-1], meas.vals,  color=meas.color,  width=bin_width,  align='edge')
       ax[index].set_ylabel(meas.y_label)
       if meas.max_val is not None:
              ax[index].set_ylim(0, meas.max_val)

plt.show()