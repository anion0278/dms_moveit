#! /bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

cd ~/ws_moveit
catkin build --jobs 3
