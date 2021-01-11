#! /bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catking_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build --jobs 3 -DCATKIN_ENABLE_TESTING=0
