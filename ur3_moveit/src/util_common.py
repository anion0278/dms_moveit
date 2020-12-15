import sys
from datetime import datetime

def print_all_args():
     for arg, i in zip(sys.argv, range(len(sys.argv))):
            print("Arg [%s]: %s" % (i, arg))

def get_time_str():
     return datetime.now().strftime("%H:%M:%S.%f")[:-3]