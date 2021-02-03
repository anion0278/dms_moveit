import sys
from datetime import datetime
import rospy
import numpy as np
from bondpy import bondpy

def print_all_args():
     for arg, i in zip(sys.argv, range(len(sys.argv))):
            print("Arg [%s]: %s" % (i, arg))

def get_time_str():
     return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def get_param(param_name):
     return rospy.get_param(param_name)

def set_param(param_name, value):
     rospy.set_param(param_name, value)

def has_nan_values(array):
     return np.isnan(np.sum(array))