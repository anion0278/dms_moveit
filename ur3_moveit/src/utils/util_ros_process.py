#!/usr/bin/python

import time
import rospy
import os
import subprocess
import psutil
import rosgraph
import rosservice


def run_process_async(command, shell=True):
    return subprocess.Popen(command, shell=shell)


def run_process_sync(command):
    os.system(command)


def kill_processes_by_name(process_name):
    for proc in psutil.process_iter():
        if proc.name() == process_name:
            proc.kill()
            print("Killed process: %s" % proc.name())

def kill_processes_by_contained_name(process_name_part):
    for proc in psutil.process_iter():
        if process_name_part in proc.name():
            proc.kill()
            print("Killed process: %s" % proc.name())

def start_roscore():
    process = run_process_async("roscore")
    rospy.wait_for_service("/rosout/get_loggers")


def is_roscore_running():
    return rosgraph.is_master_online()

def kill_ros_graph():
    print("Destroying rosgraph!")
    run_process_sync("rosnode kill -a")


def wait_for_services_or_timeout(services, timeout, timeout_callback=None):
    time_spent = 0
    while timeout >= time_spent and all(not is_rosservice_running(x) for x in services):
        time.sleep(1)
        time_spent += 1
    if (time_spent >= gazebo_timeout):
        print("Time is up!")
        timeout_callback()
        raise RuntimeError("Timeout!")


def is_rosservice_running(service_name):
    services = rosservice.get_service_list()
    return service_name in services
