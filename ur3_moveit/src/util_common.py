import sys

def print_all_args():
     for arg, i in zip(sys.argv, range(len(sys.argv))):
            print("Arg [%s]: %s" % (i, arg))